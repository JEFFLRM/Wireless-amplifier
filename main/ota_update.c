/**
 * ota_update.c - WiFi OTA 固件升级 (ESP-IDF esp_ota_ops + esp_http_server)
 *
 * 方案: 接收端在工作模式下(已连 WiFi)启动一个轻量 HTTP 服务, 端口 8080。
 *       PC/手机浏览器打开 http://<接收端IP>:8080/ , 选择编译产物
 *       build/RECEIVER_HIFI.bin 上传, 设备写入"下一个" OTA app 分区,
 *       校验通过后设置启动分区并自动重启, 新固件生效。
 *
 * 进度: 采用"分块上传"协议 —— 浏览器把固件切成 4KB 块, 逐块 POST /ota_chunk,
 *       服务端每写一块返回 {"received","total","pct"} JSON, 页面据此更新进度条。
 *       (一次 POST 整包无法获得中途写入进度, 故分块; 块间串行, 顺序写入)
 *
 * 分区要求 (partitions.csv, 需 8MB flash):
 *   nvs / otadata / phy_init / ota_0 / ota_1
 *   首次烧录时把当前固件写到 ota_0 (0x20000), bootloader 会根据 otadata 选择。
 *
 * 安全: 只接受 esp_ota_end 校验通过的镜像 (含 magic/校验); 上传中断或校验失败
 *       时 esp_ota_abort, 原运行分区不受影响, 设备继续跑旧固件。
 *       浏览器端失败时调 /ota_abort 复位状态, 可重新选择固件再试。
 */
#include <stdio.h> /* snprintf */
#include <string.h> /* memset/strlen */
#include "esp_log.h" /* ESP_LOGx */
#include "esp_err.h" /* esp_err_t */
#include "esp_http_server.h" /* HTTP 服务器 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_ota_ops.h" /* OTA API (esp_ota_begin/write/end/set_boot_partition) */
#include "esp_system.h" /* esp_restart */
#include "freertos/FreeRTOS.h" /* FreeRTOS */
#include "freertos/task.h" /* vTaskDelay */
#include "esp_heap_caps.h" /* heap_caps_malloc (OTA 缓冲放内部 RAM) */
#include "ota_update.h" /* 本模块接口 */

#define TAG "ota" /* 日志标签 */

#define OTA_HTTP_PORT       8080 /* OTA HTTP 端口 (避开音频 5004 / 发现广播 6000) */
#define OTA_BUF_SZ          4096 /* 分块大小: 每次网络接收+flash 写入 4KB */
#define OTA_REBOOT_DELAY_MS 1500 /* 升级成功后延时重启, 让 HTTP 响应先发完 */

static httpd_handle_t s_server = NULL; /* HTTP 服务句柄 */

/* OTA 会话状态 (块与块之间保持, 全局共享) */
static const esp_partition_t *s_ota_part   = NULL; /* 目标 OTA 分区 */
static esp_ota_handle_t       s_ota_handle = 0;   /* OTA 会话句柄 */
static int                    s_ota_total   = 0;   /* 固件总字节数 */
static int                    s_ota_received = 0;  /* 已写入字节数 */
static bool                   s_ota_active  = false; /* 升级会话进行中 (禁止并发) */

/* 上传页面: 分块上传 + 进度条 + FEC缓存/环形缓冲/定时播出参数 */
static const char OTA_HTML[] =
"<!DOCTYPE html><html lang=\"zh-CN\"><head><meta charset=\"utf-8\">"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
"<title>7CH Receiver OTA</title><style>"
"body{font-family:system-ui,Arial,sans-serif;max-width:780px;margin:40px auto;padding:0 16px;color:#222}"
"h2{color:#0a7}h2 span{font-size:14px;color:#666}"
"h3{margin-bottom:4px}h3 span{font-size:13px;color:#666;font-weight:normal}"
"input,button{font-size:15px}button{padding:8px 18px;background:#0a7;color:#fff;border:0;border-radius:6px;cursor:pointer}"
"button:disabled{background:#999}"
".wrap{background:#f0f0f0;border-radius:6px;overflow:hidden;margin:12px 0;height:24px}"
".bar{width:0%;height:100%;background:linear-gradient(90deg,#08f,#0a7);transition:width .15s}"
"#pct{font-weight:bold;color:#0a7}"
"pre{background:#f5f5f5;padding:12px;border-radius:6px;white-space:pre-wrap;min-height:60px;font-size:13px}"
".params{display:grid;grid-template-columns:repeat(auto-fill,minmax(170px,1fr));gap:10px 14px;margin:10px 0 14px}"
".params label{display:flex;flex-direction:column;font-size:13px;color:#444}"
".params input{padding:5px 7px;font-size:14px;border:1px solid #ccc;border-radius:4px;margin-top:3px}"
"em{font-style:normal;color:#c60;font-size:11px}"
".hint{font-size:13px;color:#666;margin:6px 0}"
"</style></head><body>"
"<h2>7CH 接收端固件升级 <span>WiFi OTA · 分块上传</span></h2>"
"<p>选择编译产物 <code>build/RECEIVER_HIFI.bin</code> 后点击上传, 设备校验通过会自动重启。</p>"
"<input type=\"file\" id=\"f\" accept=\".bin\"><br><br>"
"<button id=\"b\" onclick=\"up()\">上传并升级</button>"
"<div class=\"wrap\"><div class=\"bar\" id=\"bar\"></div></div>"
"<p>进度: <span id=\"pct\">0%</span> <span id=\"byts\" style=\"color:#888\"></span></p>"
"<pre id=\"log\">等待选择固件...</pre>"
"<hr>"
"<h3>FEC 缓存 / 环形缓冲参数 <span>在线调整, 保存到 NVS</span></h3>"
"<p class=\"hint\">帧 = 1ms。实际延迟 ≈ 起播水位 + DMA 深度。"
"FEC 缓存槽数需 ≥ 起播水位 + FEC 组记录 + 8。</p>"
"<div class=\"params\">"
"<label>FEC 缓存槽数 <em>重启生效</em><input id=\"cc\" type=\"number\" min=\"24\" max=\"256\"></label>"
"<label>FEC 组记录数 <em>重启生效</em><input id=\"cg\" type=\"number\" min=\"2\" max=\"32\"></label>"
"<label>起播水位 (帧)<input id=\"cw\" type=\"number\" min=\"4\" max=\"160\"></label>"
"<label>积压上限 (帧)<input id=\"ch\" type=\"number\" min=\"5\" max=\"200\"></label>"
"<label>裁剪目标 (帧)<input id=\"ct\" type=\"number\" min=\"1\" max=\"200\"></label>"
"<label>环形容量 (帧) <em>重启生效</em><input id=\"cr\" type=\"number\" min=\"16\" max=\"400\"></label>"
"</div>"
"<h3>定时播出参数 <span>在线调整, 保存到 NVS</span></h3>"
"<p class=\"hint\">播放时钟 late = (时钟 - ts) + DMA - 播放前瞻; late 超过迟到阈值即丢包。"
"标 <em>重启生效</em> 的项需重启后加载。</p>"
"<div class=\"params\">"
"<label>播放前瞻 (ms)<input id=\"cl\" type=\"number\" min=\"0\" max=\"500\"></label>"
"<label>DMA 深度 (ms) <em>重启生效</em><input id=\"cd\" type=\"number\" min=\"2\" max=\"32\" step=\"2\"></label>"
"<label>迟到丢弃阈值 (ms)<input id=\"cdl\" type=\"number\" min=\"1\" max=\"200\"></label>"
"<label>排水静音上限 (帧)<input id=\"cdn\" type=\"number\" min=\"1\" max=\"500\"></label>"
"<label>缺口跳转阈值 (包)<input id=\"cgj\" type=\"number\" min=\"1\" max=\"2000\"></label>"
"<label>墙钟判定容差 (ms)<input id=\"cts\" type=\"number\" min=\"100\" max=\"600000\"></label>"
"<label>起播等对时上限 (ms)<input id=\"csw\" type=\"number\" min=\"0\" max=\"60000\"></label>"
"<label>NTP 刷新周期 (ms) <em>重启生效</em><input id=\"cnt\" type=\"number\" min=\"15000\" max=\"3600000\"></label>"
"</div>"
"<button onclick=\"saveCfg()\">保存参数</button>"
"<span id=\"cmsg\" style=\"margin-left:10px;font-size:13px;color:#666\"></span>"
"<script>"
"var CH=4096;"
"function el(id){return document.getElementById(id);}"
"async function up(){"
"var f=el('f').files[0],total=f.size,off=0,ok=true;"
"var bar=el('bar'),pct=el('pct'),byts=el('byts'),log=el('log'),b=el('b');"
"if(!f){log.textContent='请先选择 .bin 固件文件';return;}"
"if(total<100000||total>4000000){log.textContent='文件大小异常: '+total+' 字节 (应为 1M~4M)';return;}"
"b.disabled=true;bar.style.width='0%';pct.textContent='0%';byts.textContent='';"
"log.textContent='开始升级: '+f.name+' ('+total+' 字节, 共 '+Math.ceil(total/CH)+' 块)...';"
"try{"
"while(off<total){"
"var end=Math.min(off+CH,total);"
"var buf=await f.slice(off,end).arrayBuffer();"
"var r=await fetch('/ota_chunk',{method:'POST',headers:{'X-Offset':off,'X-Total':total},body:buf});"
"if(!r.ok){throw new Error(await r.text());}"
"var j=await r.json();"
"off=j.received;"
"var pr=Math.min(100,j.pct);"
"bar.style.width=pr+'%';pct.textContent=Math.round(pr)+'%';"
"byts.textContent=j.received+' / '+j.total+' 字节';"
"if(j.received>=j.total)break;"
"}"
"if(!ok){throw new Error('aborted');}"
"bar.style.width='100%';pct.textContent='100%';"
"var d=await fetch('/ota_done',{method:'POST'});"
"log.textContent+='\\n'+await d.text();"
"log.textContent+='\\n升级成功, 设备将在 1~2 秒后重启。请稍候, 重新打开本页面确认版本。';"
"}catch(e){"
"log.textContent+='\\n[失败] '+e;"
"try{await fetch('/ota_abort',{method:'POST'});}catch(_){}"
"log.textContent+='\\n已复位, 可重新选择固件再试。';"
"b.disabled=false;"
"}"
"}"
"var MAP={cc:'cache',cg:'group',cw:'wmark',ch:'high',ct:'trim',cr:'ring',"
"cl:'lead',cd:'dma',cdl:'late',cdn:'drain',cgj:'gap',cts:'ts_sane',csw:'sync_wait',cnt:'ntp'};"
"async function loadCfg(){"
"try{"
"var r=await fetch('/cfg');var j=await r.json();"
"for(var id in MAP){el(id).value=j[MAP[id]];}"
"}catch(e){el('cmsg').textContent='读取参数失败';}"
"}"
"async function saveCfg(){"
"var m=el('cmsg');m.style.color='#666';m.textContent='保存中...';"
"var q=[];for(var id in MAP){q.push(MAP[id]+'='+encodeURIComponent(el(id).value));}"
"try{"
"var r=await fetch('/cfg?'+q.join('&'),{method:'POST'});var t=await r.text();"
"m.style.color=r.ok?'#0a7':'#c00';m.textContent=t;"
"}catch(e){m.style.color='#c00';m.textContent='请求失败: '+e;}"
"}"
"loadCfg();"
"</script></body></html>";

/* GET / : 返回上传页面 */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8"); /* 声明 HTML 与编码 */
    return httpd_resp_send(req, OTA_HTML, HTTPD_RESP_USE_STRLEN); /* 发送页面 */
}

/* 读取请求头数值 (无则返回 def) */
static int hdr_int(httpd_req_t *req, const char *name, int def)
{
    int len = httpd_req_get_hdr_value_len(req, name); /* 头部长度 */
    if (len <= 0 || len > 16) return def; /* 不存在或过长 */
    char buf[24]; /* 数值缓冲 */
    if (httpd_req_get_hdr_value_str(req, name, buf, sizeof(buf)) != ESP_OK) return def; /* 读取失败 */
    return atoi(buf); /* 转整数 */
}

/* POST /ota_chunk : 写入一块固件数据, 返回进度 JSON
 * 首块 (X-Total 携带且会话未激活) 初始化 OTA 会话; 块间串行保证写入顺序 */
static esp_err_t ota_chunk_handler(httpd_req_t *req)
{
    /* ====== 进入 flash 写入路径前: 提升当前 HTTP 任务到最高业务优先级, 同时让 receiver 的 i2s/net 让路 ====== */
    UBaseType_t prev_pri = uxTaskPriorityGet(NULL); /* 记录原优先级 (默认 HTTP 任务 5) */
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1); /* 提到次最高 (保留最高给系统 IDLE/软中断) */
    extern void receiver_ota_priority(bool pause); /* receiver.c 内: 降/升 i2s/net/disc/stat 优先级 */
    receiver_ota_priority(true); /* 降其他任务到 1, 给 flash 写总线让路 */

    int total_hdr = hdr_int(req, "X-Total", 0); /* 本次上传总字节数 (首块必带) */
    esp_err_t rc = ESP_OK;

    if (!s_ota_active) { /* 会话未开始: 初始化 */
        if (total_hdr <= 0) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing X-Total header"); rc = ESP_FAIL; goto cleanup; }
        s_ota_part = esp_ota_get_next_update_partition(NULL);
        if (!s_ota_part) { ESP_LOGE(TAG, "no OTA partition"); httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no OTA partition"); rc = ESP_FAIL; goto cleanup; }
        if ((uint32_t)total_hdr > s_ota_part->size) { ESP_LOGE(TAG, "firmware %d bytes > partition %u", total_hdr, (unsigned)s_ota_part->size); httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "firmware too large"); rc = ESP_FAIL; goto cleanup; }
        esp_err_t err = esp_ota_begin(s_ota_part, OTA_SIZE_UNKNOWN, &s_ota_handle);
        if (err != ESP_OK) { ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err)); httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ota begin failed"); rc = ESP_FAIL; goto cleanup; }
        s_ota_total   = total_hdr;
        s_ota_received = 0;
        s_ota_active  = true;
        ESP_LOGW(TAG, "OTA start: %d bytes -> partition '%s' @0x%08lx", s_ota_total, s_ota_part->label, (unsigned long)s_ota_part->address);
    }

    uint8_t buf[OTA_BUF_SZ];
    int remaining = req->content_len;
    if (remaining > OTA_BUF_SZ) { ESP_LOGE(TAG, "chunk %d > buf %d", remaining, OTA_BUF_SZ); esp_ota_abort(s_ota_handle); s_ota_active = false; httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "chunk too large"); rc = ESP_FAIL; goto cleanup; }

    int got = 0;
    while (got < remaining) {
        int len = httpd_req_recv(req, (char *)buf + got, remaining - got);
        if (len == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (len <= 0) { ESP_LOGE(TAG, "chunk recv failed at %d/%d", s_ota_received, s_ota_total); esp_ota_abort(s_ota_handle); s_ota_active = false; httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "upload interrupted"); rc = ESP_FAIL; goto cleanup; }
        got += len;
    }

    esp_err_t err = esp_ota_write(s_ota_handle, buf, got);
    if (err != ESP_OK) { ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err)); esp_ota_abort(s_ota_handle); s_ota_active = false; httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "flash write failed"); rc = ESP_FAIL; goto cleanup; }

    s_ota_received += got;
    int pct = s_ota_total > 0 ? (int)((int64_t)s_ota_received * 100 / s_ota_total) : 0;
    char resp[96];
    snprintf(resp, sizeof(resp), "{\"received\":%d,\"total\":%d,\"pct\":%d}", s_ota_received, s_ota_total, pct);
    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_sendstr(req, resp);
    if (s_ota_received % (256 * 1024) < got) { ESP_LOGI(TAG, "OTA progress: %d/%d (%d%%)", s_ota_received, s_ota_total, pct); }

cleanup:
    receiver_ota_priority(false); /* 恢复 receiver 各任务原优先级 */
    vTaskPrioritySet(NULL, prev_pri); /* 恢复 HTTP 任务原优先级 */
    return rc;
}

/* POST /ota_done : 校验镜像、设置启动分区并重启 */
static esp_err_t ota_done_handler(httpd_req_t *req)
{
    if (!s_ota_active) { /* 没有进行中的会话 */
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no OTA session"); /* 400 */
        return ESP_FAIL; /* 返回失败 */
    } /* 无会话分支结束 */

    esp_err_t err = esp_ota_end(s_ota_handle); /* 结束会话并校验镜像 (magic/校验和) */
    if (err != ESP_OK) { /* 镜像校验失败 */
        ESP_LOGE(TAG, "esp_ota_end failed (invalid image): %s", esp_err_to_name(err)); /* 日志 */
        s_ota_active = false; /* 复位 */
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "image validation failed (wrong file?)"); /* 400 */
        return ESP_FAIL; /* 返回失败 */
    } /* 校验失败分支结束 */

    err = esp_ota_set_boot_partition(s_ota_part); /* 设置下次启动分区 */
    if (err != ESP_OK) { /* 设置失败 */
        ESP_LOGE(TAG, "set_boot_partition failed: %s", esp_err_to_name(err)); /* 日志 */
        s_ota_active = false; /* 复位 */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "set boot partition failed"); /* 500 */
        return ESP_FAIL; /* 返回失败 */
    } /* 设置失败分支结束 */

    ESP_LOGW(TAG, "OTA success: %d bytes -> '%s', rebooting...", s_ota_received, s_ota_part->label); /* 日志 */
    s_ota_active = false; /* 复位 (重启前清状态) */

    httpd_resp_set_type(req, "text/plain; charset=utf-8"); /* 纯文本 */
    httpd_resp_sendstr(req, "ok: firmware written, device rebooting..."); /* 先回响应让页面显示 */

    vTaskDelay(pdMS_TO_TICKS(OTA_REBOOT_DELAY_MS)); /* 等响应发完 */
    esp_restart(); /* 重启进入新固件 */
    return ESP_OK; /* 不会到达 */
}

/* POST /ota_abort : 放弃本次升级, 复位会话状态 (浏览器端失败时调用, 允许重试) */
static esp_err_t ota_abort_handler(httpd_req_t *req)
{
    if (s_ota_active) { /* 会话进行中才需要放弃 */
        esp_ota_abort(s_ota_handle); /* 放弃写入 */
        s_ota_active = false; /* 复位 */
        ESP_LOGW(TAG, "OTA aborted by client at %d/%d", s_ota_received, s_ota_total); /* 日志 */
    } /* 会话分支结束 */
    httpd_resp_set_type(req, "text/plain; charset=utf-8"); /* 纯文本 */
    httpd_resp_sendstr(req, "ok: aborted, can retry"); /* 返回结果 */
    return ESP_OK; /* 返回成功 */
}

/* ====== FEC缓存 / 环形缓冲 / 定时播出参数在线调整 (/cfg) ====== */
/* 从 query string 读整数; 键存在返回 true 并写 *out */
static bool q_get_int(char *qs, const char *key, int *out)
{
    char v[16]; /* 数值缓冲 */
    if (httpd_query_key_value(qs, key, v, sizeof(v)) != ESP_OK) return false; /* 无该键 */
    *out = atoi(v); /* 转整数 */
    return true;
}

/* GET /cfg : 返回当前全部参数 JSON */
static esp_err_t cfg_get_handler(httpd_req_t *req)
{
    receiver_tuning_t t; /* 当前生效参数 */
    receiver_tuning_get(&t); /* 取当前生效值 */
    char resp[384]; /* JSON 缓冲 */
    snprintf(resp, sizeof(resp),
             "{\"cache\":%d,\"group\":%d,\"wmark\":%d,\"high\":%d,\"trim\":%d,\"ring\":%d,"
             "\"lead\":%d,\"dma\":%d,\"late\":%d,\"drain\":%d,\"gap\":%d,"
             "\"ts_sane\":%d,\"sync_wait\":%d,\"ntp\":%d}",
             t.cache_sz, t.group_rec, t.wmark, t.high, t.trim, t.ring,
             t.lead_ms, t.dma_ms, t.late_drop_ms, t.drain_fill, t.gap_jump,
             t.ts_sane_ms, t.sync_wait_ms, t.ntp_interval_ms);
    httpd_resp_set_type(req, "application/json; charset=utf-8"); /* JSON 类型 */
    return httpd_resp_sendstr(req, resp); /* 返回 */
}

/* POST /cfg?cache=&group=&wmark=&high=&trim=&ring=&lead=&dma=&late=&drain=&gap=&ts_sane=&sync_wait=&ntp=
 * 缺省键沿用当前值; 可热改项立即生效, 缓存/组记录/环深/DMA/NTP 重启生效 */
static esp_err_t cfg_post_handler(httpd_req_t *req)
{
    char qs[512]; /* query string 缓冲 (14 个键) */
    if (httpd_req_get_url_query_str(req, qs, sizeof(qs)) != ESP_OK) { /* 无 query */
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing query parameters");
        return ESP_FAIL; /* 结束 */
    }

    receiver_tuning_t t; /* 以当前值为基础, 仅覆盖请求里出现的键 */
    receiver_tuning_get(&t);
    q_get_int(qs, "cache",     &t.cache_sz);
    q_get_int(qs, "group",     &t.group_rec);
    q_get_int(qs, "wmark",     &t.wmark);
    q_get_int(qs, "high",      &t.high);
    q_get_int(qs, "trim",      &t.trim);
    q_get_int(qs, "ring",      &t.ring);
    q_get_int(qs, "lead",      &t.lead_ms);
    q_get_int(qs, "dma",       &t.dma_ms);
    q_get_int(qs, "late",      &t.late_drop_ms);
    q_get_int(qs, "drain",     &t.drain_fill);
    q_get_int(qs, "gap",       &t.gap_jump);
    q_get_int(qs, "ts_sane",   &t.ts_sane_ms);
    q_get_int(qs, "sync_wait", &t.sync_wait_ms);
    q_get_int(qs, "ntp",       &t.ntp_interval_ms);

    if (receiver_tuning_set(&t) != ESP_OK) { /* 校验 + 保存 */
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, /* 非法参数 */
                            "invalid: 4<=wmark<high<=200, 1<=trim<=high, high+8<=ring<=400, "
                            "group 2..32, cache>=wmark+group+8 && <=256, lead 0..500, "
                            "dma even 2..32, late 1..200, drain 1..500, gap 1..2000, "
                            "ts_sane 100..600000, sync_wait 0..60000, ntp 15000..3600000");
        return ESP_FAIL; /* 结束 */
    }
    char resp[256]; /* 提示缓冲 */
    snprintf(resp, sizeof(resp),
             "saved: cache=%d group=%d wmark=%d high=%d trim=%d ring=%d lead=%d dma=%d "
             "late=%d drain=%d gap=%d ts_sane=%d sync_wait=%d ntp=%d "
             "(cache/group/ring/dma/ntp take effect after reboot)",
             t.cache_sz, t.group_rec, t.wmark, t.high, t.trim, t.ring, t.lead_ms,
             t.dma_ms, t.late_drop_ms, t.drain_fill, t.gap_jump,
             t.ts_sane_ms, t.sync_wait_ms, t.ntp_interval_ms); /* 组提示 */
    httpd_resp_set_type(req, "text/plain; charset=utf-8"); /* 纯文本 */
    return httpd_resp_sendstr(req, resp); /* 返回 */
}

/* 启动 OTA HTTP 服务 */
esp_err_t ota_update_start(void)
{
    if (s_server) return ESP_OK; /* 已启动, 幂等返回 */

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG(); /* 默认配置 */
    cfg.server_port      = OTA_HTTP_PORT; /* 监听 8080 */
    cfg.ctrl_port        = OTA_HTTP_PORT + 1; /* 控制端口错开, 避免冲突 */
    cfg.stack_size       = 8192; /* 服务器任务栈 (分块上传处理用) */
    cfg.max_uri_handlers = 6; /* 6 个路由: /, /ota_chunk, /ota_done, /ota_abort, /cfg(GET+POST) */
    cfg.lru_purge_enable = true; /* 连接数满时清理最久未用 */
    cfg.recv_wait_timeout = 20; /* 单块接收超时 20s */
    cfg.send_wait_timeout = 20; /* 发送超时 20s */
    /* ESP-IDF v6 httpd_config_t 无 priority 字段, 改用 httpd_req_handle 内部临时提优先级 */

    esp_err_t err = httpd_start(&s_server, &cfg); /* 启动 HTTP 服务 */
    if (err != ESP_OK) { /* 启动失败 */
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(err)); /* 日志 */
        s_server = NULL; /* 清空句柄 */
        return err; /* 返回错误 */
    } /* 启动失败分支结束 */

    static const httpd_uri_t uri_root  = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler }; /* 首页 */
    static const httpd_uri_t uri_chunk = { .uri = "/ota_chunk", .method = HTTP_POST, .handler = ota_chunk_handler }; /* 分块写入 */
    static const httpd_uri_t uri_done  = { .uri = "/ota_done", .method = HTTP_POST, .handler = ota_done_handler }; /* 完成校验 */
    static const httpd_uri_t uri_abort = { .uri = "/ota_abort", .method = HTTP_POST, .handler = ota_abort_handler }; /* 放弃 */
    static const httpd_uri_t uri_cfg_get  = { .uri = "/cfg", .method = HTTP_GET,  .handler = cfg_get_handler };  /* 播放参数读取 */
    static const httpd_uri_t uri_cfg_post = { .uri = "/cfg", .method = HTTP_POST, .handler = cfg_post_handler }; /* 播放参数设置 */
    httpd_register_uri_handler(s_server, &uri_root); /* 注册首页 */
    httpd_register_uri_handler(s_server, &uri_chunk); /* 注册分块 */
    httpd_register_uri_handler(s_server, &uri_done); /* 注册完成 */
    httpd_register_uri_handler(s_server, &uri_abort); /* 注册放弃 */
    httpd_register_uri_handler(s_server, &uri_cfg_get); /* 注册参数读取 */
    httpd_register_uri_handler(s_server, &uri_cfg_post); /* 注册参数设置 */

    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL); /* 检查 OTA 分区是否可用 */
    if (next) { /* 分区存在 */
        ESP_LOGI(TAG, "OTA server ready: http://<device-ip>:%d/ (target '%s' @0x%08lx, %u KB)", /* 日志 */
                 OTA_HTTP_PORT, next->label, (unsigned long)next->address, (unsigned)(next->size / 1024)); /* 分区信息 */
    } else { /* 分区缺失: 服务照常启动, 上传时返回错误 */
        ESP_LOGE(TAG, "OTA server started but NO ota partition: check partitions.csv"); /* 警告 */
    } /* 分区检查结束 */
    return ESP_OK; /* 返回成功 */
}
