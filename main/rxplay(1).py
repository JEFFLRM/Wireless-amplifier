#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rxplay.py v5.5.0 - Windows 10 UDP 音频接收/播放器（esp32 协议, audio_router 契约 v1.0/v1.1/v1.2/v1.3）

  纯 Python 标准库(tkinter 为 Windows 自带), 无需 pip 安装任何东西。
  组播 join / 按需单播 → 解析 RTP(PT=96 数据 / 97 FEC 跳过) → 交错载荷抽取选定声道
  → S16 送 winmm waveOut 实时播放; seq 断洞补静音 / 迟到包丢弃 / 流重启重同步 /
  2s 无包自动收声, 统计 rx/fec/late/loss 到达率。
  v4: 输出恒为立体声(无需重开 waveOut), GUI 运行中可随时切换监听任意声道。
  v5: 无音频输出设备自动转"仅统计模式"(收包/到达率, 不播放)。
  v5.1 播放延迟优化: timeBeginPeriod(1) 换 1ms 系统时钟粒度(sleep 不再实睡 15.6ms)、
      块粒度 10→5ms、在途上限按延迟档位 --profile low/normal/stable(默认 low=40ms 在途,
      原 60ms)、软限追沿(超限每 100ms 微量丢 1 块)替代 300ms 硬重置抗时钟漂移。
  v5.2 conv() 算法重写: 逐帧 Python 循环 → 全 C 级字节跨步切片+struct.unpack
      (L/R 档 ~7x, 8ch 混音 ~2.3x, 输出逐字节等价已验证)。
  v5.2.1 多声道混音改声像定位下混(ITU-R BS.775 风格)。
  v5.2.2 播放干净/流畅优先: 修 waveOut header 泄漏、硬重置阈值 800ms、
      软限分级追沿、欠载自适应加深缓冲、流结束 flush 联动。
  v5.2.3 UI 布局重排: 参数区两行、控制栏上移、监听区快捷钮、异常着色。
  v5.3.0 音质与体验:
      ① 下混改标准 ITU 系数(C/LFE 0.707/0.5, 环绕 0.707)+ 逐包峰值限幅器
         (超限即时压、每包 +0.2% 恢复) —— 修"多个满幅声道相加硬 clamp 削波"
         导致的沙沙, 正常电平不受影响、不改变音色;
      ② 丢包补静音上限 200ms: 更长的断洞不再一次性灌静音(会把延迟顶起来),
         直接重锚时间轴继续播;
      ③ 统计新增"限制/峰值dBFS", 一眼看出是否在削波;
      ④ UI: 空格开始/停止、1~8 单声道、A 全选、C 清空 快捷键; 参数自动记忆
         (rxplay_cfg.json); **网卡IP 自动探测**(下拉列出全部网卡+网卡名, 默认出口优先,
         可手填/↻ 重探); 窗口大小按内容自适应并记住上次尺寸; 高 DPI 适配; 版本/状态标题;
      ⑤ winmm 惰性初始化: 非 Windows 或无声卡时自动进"仅统计模式", 便于跨平台验证。
  v5.3.1 v1.2 发现包 + 停止/静音联动板端:
      ① 发现包改 21B(v1.2 新增 tx_enable), 与新版音箱固件/arsend 对齐;
      ② **按"停止"** → 板端 1ms 内停发该机(表项保留不老化), 再点开始无需重注册;
         发包在 worker 退出(winmm 设备已关)后 250ms 由后台线程执行;
      ③ **按"清空(静音)"** → 同样置 tx_enable=0 让板端停发省空口, 选回声道即复流;
      ④ 修闪退: player 退出前先 waveOutReset + 逐块 UnprepareHeader 再释放缓冲
         (原先 GC 先释放、驱动仍持有指针 -> 堆损坏 -> 任意线程分配内存即原生 AV);
      ⑤ 修 NCH=0(清空声道)时 conv() 除零崩溃(帧长按 1 声道估, 输出等长静音);
      ⑥ 停止路径新增 Tk 回调异常/原生崩溃(faulthandler)落盘, 便于事后定位。
  v5.4.0 (本版) v1.3 批量流(doc/协议升级_批量包化_v1.3_草案):
      ① 发现包 [14] 置 caps=1(bit1): 新发端(AR_TX_BATCH>0)对本机 1~2 声道订阅
         改发 PT=100/101 批量流(2/4ms 大包); 旧发端 tx?1:0 把 bit1 布尔化吞掉,
         新旧发端随意混用, 完全兼容;
      ② RTP 解析认 PT=100(媒体, 载荷按实际长度自适应)/101(FEC); 断洞补静音
         本来就按"当前包换算长度"计(gap × 本包时长), 4ms 包丢 1 包自动补 4ms,
         时间轴语义不变, 无需特判;
      ③ 统计加 b4=(PT=100 包计数), 首个批量包打一行参数(载荷长度/折算 ms);
      ④ NTP 对时探测(方案 A F1 同款算法): 首包后向流源 IP :123 周期对时(30s),
         打印 offset/rtt 并上统计行(ntp=+x.xms) —— 发端 ntpd 装好后本工具即可
         验收 |offset|<5ms, 不必等音箱固件; --no-ntp 可关;
      ⑤ NTP 端到端延迟: e2e = (本机now+offset) - ts/48000, 统计行
         'e2e=avg/min/max(+buf=播放口径)' —— 到达口径=发端采样→PC收到,
         播放口径≈到达+缓冲水位(声学出声另有系统混音 ~30-50ms 测不到);
         ts 回绕用模差法天然正确, >1s 跳变(发端重锚)自动重置会话;
      ⑥ --profile ultra(10+20ms): 有线/低抖动台架的低延迟档;
      ⑦ 起播跳直播沿: 设备开启期间接收侧积压(实测 ~550ms)不再靠软限追 30s,
         起播直接丢弃到只留预攒, 秒进稳态(代价=跳过开头一小段)。
  v5.5.0 (本版) WASAPI 独占输出: PC 端 ≈55ms 端到端极限档的最后一块拼图。
      ① winmm 波形 API 必经 Windows 共享混音引擎(+30~50ms 且不可控);
         WASAPI 独占模式(ctypes 直调 COM: 设备枚举→IAudioClient→IAudioRenderClient)
         事件驱动直写设备缓冲, 出声延迟 ≈ 设备周期(默认 ~10ms)×缓冲倍数;
      ② --wasapi auto(默认)/on/off: auto=独占失败(设备被占/格式不符)自动回退
         winmm, 行为不劣化; on=强制独占(失败报错); off=行为同 v5.4;
      ③ player() 双后端: WASAPI 在途=hwo 内部 ring 字节(buffered_ms), 欠载判据
         =ring 近空, 硬重置/流结束 flush=reset() 清 ring; 软限/起播跳沿/自适应
         加深等策略与 winmm 完全一致;
      ④ 独占期间系统混音静音(其他程序无声), 停止后自动恢复; GUI"延迟档位"
         下拉补"极限"(=ultra 10+20ms)。

用法 (Windows 10 + Python 3.x, 安装时勾选 Add to PATH):
  python rxplay.py                          # 无参数 = GUI 图形界面
  python rxplay.py --gui                    # 强制 GUI
  python rxplay.py --rate 192000            # CLI 模式(带任一参数即 CLI): 192k 定案档
  python rxplay.py --rate 96000             # 96k 兼容档(esp32 现行固件)
  python rxplay.py --ch 2,3                 # 播 C/LFE 声道对 → L/R
  python rxplay.py --ch 4                   # 单声道复制到双耳
  python rxplay.py --raw --group none       # 裸流(无 RTP 头, 广播/单播调试档)
  python rxplay.py --fmt s32                # 现役 4 字节槽格式(默认)
  python rxplay.py --profile stable         # 网络抖动大断续时加大缓冲(默认 low 最低延迟)
  python rxplay.py --profile ultra --wasapi on   # 极限档: WASAPI 独占 + 10+20ms(端到端 ≈55ms)
  python rxplay.py --od --iface <本机IP> --ch 0,1   # v1.1 按需单播: 发发现包收专属子集流
  首次运行防火墙弹窗勾选"允许访问"。
  ⚠ 采样率必须与发送端流一致, 否则严重畸变(欠载/过速); GUI 里有醒目选择器。

声道→内容: 0/1=L/R 2/3=C/LFE 4/5=SL/SR 6/7=SBL/SBR (板端测试音 220+110*ch Hz)
"""
import argparse
import faulthandler
import json
import math
import os
import queue as queue_mod
import re
import socket
import struct
import sys
import threading
import time
import traceback
import ctypes

try:
    import audioop          # 音量增益(S16, C 实现); Python 3.13 起移除, 下面有兜底
except ImportError:
    audioop = None
from ctypes import wintypes
try:
    import numpy as _np   # 可选加速(仅 >=3 声道混音): 缺 numpy 时纯 Python 路径照跑, 不破坏零依赖承诺
except ImportError:
    _np = None

# ---------------- 播放参数(运行前按配置设置) ----------------
PORT = 5004
RATE = 48000
FMT_S24 = True
SBY = 3                              # 每样本字节数 (s24=3 / s32=4)
NCH = 8                              # 流内总声道数
OCH = 2                              # 播放声道数(恒立体声; 单声道选择=复制到 L/R)
BLOCK_MS = 5                         # 播放块粒度(细粒度=在途限流台阶小, 延迟低)
MAX_LEAD_MS = 800                    # 硬重置只留给真断流级积压; AP 组播攒批(~300ms/批)
                                     # 交给动态软限+自适应缓冲消化, 不再触发整段丢弃爆音
# 延迟档位: (起播预攒ms, waveOut在途上限ms) —— 端到端≈两者之和+系统混音引擎(~30-50ms)
PROFILES = {'ultra': (10, 20), 'low': (20, 40), 'normal': (30, 60), 'stable': (60, 150)}
                                     # ultra(2026-11-13): 有线/低抖动台架实测档, 预攒10+在途20;
                                     # 播放总延迟还有系统混音引擎 ~30-50ms, WASAPI 独占才能消(后续)
PREBUF_MS, MAX_INFLIGHT_MS = PROFILES['low']
SOFT_MARGIN_MS = 80   # 软限余量(追沿只砍到软限带内): 按档位在 worker 启动时重设,
                      # ultra=20(2026-11-13): 原固定 80 是"稳定优先"老参数, 低延迟档
                      # 下实测把稳态水位顶到 ~150ms, 而网络 rtt 仅 ~3ms, 纯属浪费

# 当前监听选择: 声道元组(有序去重)。0 个=静音 / 1 个=复制双耳 / 2 个=分送 L/R
# / >=3 个=声像定位混音(L/SL/SBL→左耳, R/SR/SBR→右耳, C/LFE→双耳×0.707)。
# 单次元组赋值即原子切换, 接收线程每包重读 → 运行中随时可换, 无需重开 waveOut。
SEL = (0, 1)
REQ = (0, 1)                          # 用户选择的契约槽位(恒存): 发现包 mask/按钮高亮/描述
OD = False                            # v1.1 按需单播模式: SEL 重映射为流内位置
SEL_EV = threading.Event()            # 声道变更事件: 唤醒 od 信标线程立即补发(不等 20s 慢心跳)
TX_OFF = [False, 0, '']               # 待发停发通知: [待发?, zone, iface]
                                      # (停止后由 worker 退出回调触发, 见 on_stop/poll)
SLOT_OF_POS = None                    # od 模式: 流内位置 j -> 契约槽位(混音 ROUTE 权重用)
# 混音限幅器状态(仅 >=3 声道混音路径): [当前增益, 限制事件数, 峰值保持] × 左耳/右耳。
# 增益在超限时立即压低、随后每包 +0.2% 恢复(1000pps, -6dB 约 0.25s 回满); 换台时复位, 避免旧衰减残留。
LIM_STATE = [[1.0, 0, 0], [1.0, 0, 0]]
MAX_SILENCE_MS = 200                  # 丢包补静音上限(超出的断洞直接重锚, 不灌静音)

def set_sel(chs):
    """chs: 0~8 个 0..7 契约槽位(多选混音)。运行中可调用。
    od 按需单播模式下 SEL 重映射为流内位置(流=请求子集升序), REQ 恒存契约槽位;
    SLOT_OF_POS 同步维护(流内位置->契约槽位, 混音 ROUTE 权重用; 非 od 恒 None)。
    NCH 必须与 SEL 同 tick 更新——脱节窗口内 SEL 长度>NCH 会让 conv 切片
    长度不齐直接抛 ValueError 崩掉接收线程(2026-10-23 实测)。"""
    global SEL, REQ, SLOT_OF_POS, NCH
    new = tuple(sorted(set(chs)))
    if new != REQ:
        REQ = new
        SEL_EV.set()                  # 立即唤醒信标线程携带新 slot_bit(定稿③: 换台不排队)
        LIM_STATE[0][0] = LIM_STATE[1][0] = 1.0   # 换台复位限幅增益, 避免旧衰减残留
    if OD:
        NCH = len(REQ)
        SEL = tuple(range(len(REQ)))
        SLOT_OF_POS = REQ
    else:
        SEL = REQ
        SLOT_OF_POS = None

def sel_desc():
    names = ['L', 'R', 'C', 'LFE', 'SL', 'SR', 'SBL', 'SBR']
    n = len(REQ)
    if n == 0:
        return '未选声道(静音)'
    if n == 1:
        return 'ch%d(%s) 复制到双耳' % (REQ[0], names[REQ[0]])
    if n == 2:
        return 'ch%d,%d(%s/%s) → L/R' % (REQ[0], REQ[1], names[REQ[0]], names[REQ[1]])
    return 'ch%s %d声道声像混音 → 立体声' % (','.join(map(str, REQ)), n)

def od_apply():
    """od 模式按当前 REQ 重排流参数(NCH=子集声道数)。换台时由信标线程调用。"""
    global NCH
    NCH = len(REQ)
    set_sel(REQ)

# ---------------- v1.1 按需单播: 发现信标(音箱侧角色, 对齐 esp32 现行) ----------------
def crc16_ccitt_false(b):
    """CRC16-CCITT-FALSE: init=0xFFFF, poly=0x1021, MSB-first(音箱发现包同款)"""
    crc = 0xFFFF
    for x in b:
        crc ^= x << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc

def _disc_pkt(zone, mask, tx, seq, ip_bytes, ts=None):
    """构造 21B 发现包(v1.2+v1.3): RTP头 + id 1B + slot_bit + tx/caps + ip 4B + CRC16 小端"""
    if ts is None:
        ts = int(time.time() * 1000) & 0xFFFFFFFF
    # [14] = bit0 tx_enable(v1.2) | bit1 caps(v1.3, 1=本机认得 PT=100/101 批量流)。
    # 旧发端读 [14] 走 tx?1:0, bit1 被吞掉照常收 1ms 流; 新发端 AR_TX_BATCH>0 时
    # 对 1~2 声道订阅改发批量大包(空口 ~60%→~15%)。
    body = (struct.pack('!BBHII', 0x80, 98, seq & 0xFFFF, ts & 0xFFFFFFFF, 0x7C41)
            + bytes([zone & 0xFF, mask & 0xFF, (1 if tx else 0) | 0x02]) + ip_bytes)
    return body + struct.pack('<H', crc16_ccitt_false(body))


def _cur_mask():
    """当前请求的 slot_bit: 空选保护 = slot0"""
    mask = 0
    for c in REQ:
        mask |= 1 << c
    return mask or 1


def _beacon_dsts(ip_int, iface):
    """发现包三个投递目标: /24 定向广播 + 受限广播 + 本机 IP 单播。
    本机 IP 这一份专供"同机自测"(发送端与接收端在同一台机器): Linux 上本机发出的
    广播不会回送给本机绑定 INADDR_ANY 的 socket, 只发广播时同机永远注册不上
    (实测 rx=0); 真实场景(PC→板/音箱跨主机)多这一份无害(本机无监听端只多丢弃一次)。"""
    return {socket.inet_ntoa(struct.pack('!I', ip_int | 0xFF)),
            '255.255.255.255', iface}


def send_tx_beacon(iface, zone, tx, n=3):
    """同步连发 n 个发现包(不等慢心跳): 停止按键时用 tx=0 让板端立即停发流。
    返回成功发出的包数; 任何异常都吞掉返回 0(调用方在 GUI 停止路径, 不许崩)。"""
    if not iface:
        return 0
    s = None
    ok = 0
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        s.bind((iface, 0))                    # 锁定出口, 源 IP 与 ip 字段一致
        ip_bytes = socket.inet_aton(iface)
        ip_int = struct.unpack('!I', ip_bytes)[0]
        dsts = _beacon_dsts(ip_int, iface)
        mask = _cur_mask()
        for i in range(max(1, n)):
            pkt = _disc_pkt(zone, mask, tx, i, ip_bytes)
            for dst in dsts:
                try:
                    s.sendto(pkt, (dst, 6000))
                    ok += 1
                except OSError:
                    pass
            time.sleep(0.02)                  # 20ms 间隔(同音箱首次上线连发)
    except Exception:
        return 0
    finally:
        if s is not None:
            try:
                s.close()
            except Exception:
                pass
    return ok


# ---------------- v1.3 配套: NTP 对时探测(方案 A, 与音箱 F1 同算法) ----------------
NTP_ERA = 2208988800                # 1900 纪元 -> 1970 epoch 的秒差

def _ntp_call(host, timeout=1.0):
    """单次 NTP 往返: 返回 (offset_ms, rtt_ms) 或 None(无应答/异常)。
    与发端 ntpd / 音箱 esp_netif_sntp 同款四时间戳:
    offset = ½[(t2−t1)+(t3−t4)], rtt = (t4−t1)−(t3−t2)。"""
    s = None
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(timeout)
        t1 = time.time()
        s.sendto(b'\x1b' + b'\x00' * 47, (host, 123))   # LI=0 VN=3 Mode=3(client)
        data, _ = s.recvfrom(128)
        t4 = time.time()
        if len(data) < 48:
            return None
        t2 = (struct.unpack('!I', data[32:36])[0]
              + struct.unpack('!I', data[36:40])[0] / 4294967296.0 - NTP_ERA)
        t3 = (struct.unpack('!I', data[40:44])[0]
              + struct.unpack('!I', data[44:48])[0] / 4294967296.0 - NTP_ERA)
        off = ((t2 - t1) + (t3 - t4)) / 2.0
        rtt = (t4 - t1) - (t3 - t2)
        return off * 1000.0, rtt * 1000.0
    except Exception:
        return None
    finally:
        if s is not None:
            try:
                s.close()
            except Exception:
                pass


def _ntp_thread(host, stop_event, log, buf):
    """周期对时(30s): 目标 = RTP 流源 IP(= 发端 = ts 时基本身, 见方案A文档 F1)。
    每轮连发 3 次取 rtt 最小的样本(SNTP 标准实践: rtt 小 ≈ 上下行对称 ≈ offset 准;
    实测 Windows 单样本 rtt 13~30ms 抖动, offset 误差可达 ±50ms, 会整体抬高 e2e)。
    首次成功打一行; 之后偏移变化 >5ms 才提示; 最新值挂 buf['ntp_ms'] 供统计行。
    发端未开 AR_NTPD(旧固件)时失败只提示一次, 之后静默重试(升级后自动恢复)。"""
    ok = fail_logged = False
    while not stop_event.is_set():
        best = None
        for _ in range(3):
            r = _ntp_call(host)
            if r and (best is None or r[1] < best[1]):
                best = r
            if stop_event.is_set():
                break
        if best:
            off, rtt = best
            prev = buf.get('ntp_ms')
            buf['ntp_ms'] = round(off, 2)
            buf['ntp_rtt_ms'] = round(rtt, 1)   # UI"网络 rtt"格: 发端↔本机往返
            if not ok:
                ok = True
                log('NTP 对时成功: 发端=%s offset=%+.1fms rtt=%.1fms(3取最小)'
                    % (host, off, rtt))
            elif prev is None or abs(off - prev) > 5.0:
                log('NTP 偏移更新: %+.1fms -> %+.1fms'
                    % (prev if prev is not None else 0.0, off))
        elif not ok and not fail_logged:
            fail_logged = True
            log('NTP 无应答(%s:123)—— 发端固件未开 AR_NTPD? 30s 后静默重试' % host)
        stop_event.wait(30.0)


def _beacon_impl(stop_event, iface, log, buf, zone):
    """广播 pt=98 发现包(21B, v1.2: RTP头 + id 1B + slot_bit + tx_enable + ip + CRC16
    小端[19..21)对前19B) 到 :6000; tx_enable 恒 1(在本线程存活期间持续要流)。
    **停止时本线程不发 tx=0** —— 见 on_stop/poll 说明: 停止瞬间 winmm 正在
    waveOutReset/Close(原生调用), 此时任何发包(内存分配)都会撞上堆损坏而 AV。
    停发通知统一在 worker 完全退出(设备已关)后由 poll 触发 send_tx_beacon(tx=0)。
    心跳(2026-10-23 音箱端改版): 无流 5s 一发(快注册), 收到流 20s 一发(降频);
    REQ 变更即换台(定稿③): 立即补发信标携带新 slot_bit, 板端下个 1ms 包切换载荷,
    seq/ts 时基不中断(不等 20s 慢心跳)。"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.bind((iface, 0))                    # 多网卡: 锁定出口, 源 IP 与 ip 字段一致
    # 双保险投递: /24 定向网段广播(常见家庭网段, 出口路由明确) + 受限广播
    # (多网卡/非常规掩码下任一生效即可; 板端 0.0.0.0:6000 两种都收)。
    # 注: ip|0xFF 假定 /24 掩码, 非 /24 网段时定向地址可能不准, 靠受限广播兜底
    ip_bytes = socket.inet_aton(iface)
    ip_int = struct.unpack('!I', ip_bytes)[0]
    dsts = _beacon_dsts(ip_int, iface)
    seq = 0
    t0 = time.time()
    last_mask = None
    last_tx = None
    while not stop_event.is_set():
        mask = _cur_mask()
        # 清空声道(REQ 空) = 静音: 置 tx_enable=0 请板端停发, 省空口/CPU;
        # 重新选上声道即回 1 复流(表项保留, 无需重注册)
        tx = 1 if REQ else 0
        changed = (mask != last_mask) or (tx != last_tx)
        if changed:
            if last_mask is not None and mask != last_mask:
                log('换台: slot_bit 0x%02x → 0x%02x (ch=%d)' %
                    (last_mask, mask, bin(mask).count('1')))
            if last_tx is not None and tx != last_tx:
                log('tx_enable %d → %d (%s)' %
                    (last_tx, tx, '恢复发送' if tx else '静音, 请求板端停发流'))
            last_mask = mask
            last_tx = tx
            od_apply()
        seq = (seq + 1) & 0xFFFF
        ts = int((time.time() - t0) * 1000) & 0xFFFFFFFF
        if stop_event.is_set():             # 停止信号已到: 绝不再发包(与 winmm 收尾并发=AV)
            break
        pkt = _disc_pkt(zone, mask, tx, seq, ip_bytes, ts)
        for dst in dsts:
            try:
                s.sendto(pkt, (dst, 6000))
            except OSError:
                pass
        # 心跳间隔: 有流 20s(10s 内收到过数据) / 无流 5s;
        # SEL_EV 被点亮相道变更 -> 提前唤醒, 下一轮立即补发(定稿③)
        if changed:
            continue
        rx_recent = time.time() - buf.get('last_pkt', 0) < 10
        if SEL_EV.wait(20.0 if rx_recent else 5.0):
            SEL_EV.clear()
    s.close()

def beacon(stop_event, iface, log, buf, zone=0):
    """线程入口: 兜底捕获一切异常——裸异常在无控制台的 pythonw 下不可见,
    且信标线程持有 socket, 异常路径必须保证线程体面退出"""
    try:
        _beacon_impl(stop_event, iface, log, buf, zone)
    except Exception as e:
        try:
            log('信标线程异常: %s' % e)
        except Exception:
            pass

def apply_cfg(port, rate, chs, fmt_s24):
    global PORT, RATE, FMT_S24, SBY
    PORT, RATE, FMT_S24 = port, rate, fmt_s24
    SBY = 3 if FMT_S24 else 4
    set_sel(chs)

def _win_adapters():
    """Windows: 用 iphlpapi.GetAdaptersAddresses 枚举**所有**网卡的 IPv4。

    纯 ctypes 无第三方依赖。返回 [(ip, 适配器名)], 失败/非 Windows 返回 []。
    相比"主机名解析"只给一个地址, 这里能列出 以太网 / WLAN / VMware / VPN
    全部网卡, 多网卡机器才能选对出口。"""
    try:
        iphlpapi = ctypes.windll.iphlpapi
    except Exception:
        return []

    class SOCKADDR_IN(ctypes.Structure):
        _fields_ = [('sin_family', ctypes.c_ushort), ('sin_port', ctypes.c_ushort),
                    ('sin_addr', ctypes.c_ubyte * 4), ('sin_zero', ctypes.c_ubyte * 8)]

    class SOCKET_ADDRESS(ctypes.Structure):
        _fields_ = [('lpSockaddr', ctypes.c_void_p), ('iSockaddrLength', ctypes.c_int)]

    class UNICAST(ctypes.Structure):
        pass
    # Length/Flags 在 C 里是 union{ULONGLONG; struct{ULONG,DWORD}} —— 4+4 与 x64 布局一致
    UNICAST._fields_ = [('Length', ctypes.c_ulong), ('Flags', ctypes.c_ulong),
                        ('Next', ctypes.POINTER(UNICAST)),
                        ('Address', SOCKET_ADDRESS)]

    class ADAPTER(ctypes.Structure):
        pass
    ADAPTER._fields_ = [('Length', ctypes.c_ulong), ('IfIndex', ctypes.c_ulong),
                        ('Next', ctypes.POINTER(ADAPTER)),
                        ('AdapterName', ctypes.c_char_p),
                        ('FirstUnicastAddress', ctypes.POINTER(UNICAST)),
                        ('FirstAnycastAddress', ctypes.c_void_p),
                        ('FirstMulticastAddress', ctypes.c_void_p),
                        ('FirstDnsServerAddress', ctypes.c_void_p),
                        ('DnsSuffix', ctypes.c_wchar_p),
                        ('Description', ctypes.c_wchar_p),
                        ('FriendlyName', ctypes.c_wchar_p)]

    AF_UNSPEC, AF_INET = 0, 2
    flags = 0x2 | 0x4 | 0x8        # SKIP_ANYCAST | SKIP_MULTICAST | SKIP_DNS_SERVER
    size = ctypes.c_ulong(16 * 1024)
    buf = ctypes.create_string_buffer(size.value)
    ret = iphlpapi.GetAdaptersAddresses(ctypes.c_ulong(AF_UNSPEC), ctypes.c_ulong(flags),
                                        None, buf, ctypes.byref(size))
    if ret == 111:                 # ERROR_BUFFER_OVERFLOW: 按返回尺寸重取一次
        buf = ctypes.create_string_buffer(size.value)
        ret = iphlpapi.GetAdaptersAddresses(ctypes.c_ulong(AF_UNSPEC), ctypes.c_ulong(flags),
                                            None, buf, ctypes.byref(size))
    if ret != 0:
        return []
    out, p = [], ctypes.cast(buf, ctypes.POINTER(ADAPTER))
    while p:
        a = p.contents
        ua = a.FirstUnicastAddress
        while ua:
            sa = ctypes.cast(ua.contents.Address.lpSockaddr,
                             ctypes.POINTER(SOCKADDR_IN)).contents
            if sa.sin_family == AF_INET:
                out.append(('.'.join(str(b) for b in sa.sin_addr), a.FriendlyName or ''))
            ua = ua.contents.Next
        p = a.Next
    return out

def local_ipv4s():
    """本机 IPv4 候选, 返回 [(ip, 网卡名)], **默认出口的排最前**。
    与 ctrl_sim.py/pcsend.py 的 local_ips 镜像维护(零依赖不抽公共模块), 改动需同步三处。

    Windows 走 GetAdaptersAddresses(能列全 VMware/VPN); 其它平台或调用失败时
    退回 UDP connect(只选路由、不发包) + 主机名解析。过滤 127.* 与 169.254.*。"""
    preferred = None
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(('8.8.8.8', 53))          # UDP connect 不产生流量
            preferred = s.getsockname()[0]
        finally:
            s.close()
    except Exception:
        pass
    out, seen = [], set()
    for ip, name in _win_adapters():
        if ip in seen or ip.startswith('127.') or ip.startswith('169.254.'):
            continue
        seen.add(ip)
        out.append((ip, name))
    if not out:                                  # 非 Windows / 枚举失败: 退回解析
        cands = ([preferred] if preferred else [])
        try:
            for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
                cands.append(info[4][0])
        except Exception:
            pass
        for ip in cands:
            if ip in seen or ip.startswith('127.') or ip.startswith('169.254.'):
                continue
            seen.add(ip)
            out.append((ip, ''))
    if preferred:
        out.sort(key=lambda t: 0 if t[0] == preferred else 1)
    return out

# ---------------- winmm waveOut (16bit PCM) ----------------
# 惰性/容错: 非 Windows 或精简 Python 下 ctypes.windll 不存在。此时 winmm=None →
# open_waveout 按"无输出设备"抛错 → run_receiver 自动进"仅统计模式",
# 脚本仍可用于到达率/丢包验收(跨平台自测), 也让 conv() 等算法可单独验证。
try:
    winmm = ctypes.windll.winmm
except Exception:
    winmm = None

class WAVEFORMATEX(ctypes.Structure):
    _fields_ = [('wFormatTag', ctypes.c_ushort), ('nChannels', ctypes.c_ushort),
                ('nSamplesPerSec', ctypes.c_uint), ('nAvgBytesPerSec', ctypes.c_uint),
                ('nBlockAlign', ctypes.c_ushort), ('wBitsPerSample', ctypes.c_ushort),
                ('cbSize', ctypes.c_ushort)]

class WAVEHDR(ctypes.Structure):
    _fields_ = [('lpData', ctypes.c_char_p), ('dwBufferLength', wintypes.DWORD),
                ('dwBytesRecorded', wintypes.DWORD), ('dwUser', ctypes.c_size_t),
                ('dwFlags', wintypes.DWORD), ('dwLoops', wintypes.DWORD),
                ('lpNext', ctypes.c_void_p), ('reserved', ctypes.c_void_p)]

# ---------------- WASAPI 独占输出(v5.5.0): 跳过共享混音引擎 ----------------
# winmm 波经 Windows 共享混音引擎(+30~50ms); WASAPI exclusive 直写设备缓冲,
# 延迟 ≈ 所选设备周期(默认 10ms)×N。失败(设备被占/格式不符)抛异常, 调用方
# 回退 winmm, 行为不劣化。纯 ctypes, 零依赖。
_W_GUID = {
    'mmdev_enum_cls': (0xbcde0395, 0xe52f, 0x467c, 0x8e3d, 0xc4579291692e),
    'mmdev_enum_iid': (0xa95664d2, 0x9614, 0x4f35, 0xa746, 0xde8db63617e6),
    'audio_client'  : (0x1cb9ad4c, 0xdbfa, 0x4c32, 0xb178, 0xc2f568a703b2),
    'audio_render'  : (0xf294acfc, 0x3146, 0x4483, 0xa7bf, 0xaddca7c260e2),
}
def _guid(t):
    """GUID 文本 5 段 (d1-d2-d3-d4-d5) → COM 内存结构(末两段=连续 8 字节大端)。
    初版漏拼 t[4](只取 16bit d4) → CLSID 错位报 0x80040154 类未注册, 已修。"""
    class _G(ctypes.Structure):
        _fields_ = [('d1', ctypes.c_ulong), ('d2', ctypes.c_ushort),
                    ('d3', ctypes.c_ushort), ('d4', ctypes.c_ubyte * 8)]
    g = _G()
    g.d1, g.d2, g.d3 = t[0], t[1], t[2]
    v = (t[3] << 48) | t[4]               # 0x8e3d<<48 | 0xc4579291692e
    for i in range(8):
        g.d4[i] = (v >> (56 - 8 * i)) & 0xFF
    return g

def _vt(this, idx, rest, atypes):
    """取 COM 对象 vtable 第 idx 项并包装成可调用(WINFUNCTYPE)。
    COM 对象内存首槽=vtable 指针, vtable 才是函数指针数组 —— 两次取址:
    对象->[0]=vtable 地址->cast->[idx]=函数地址。"""
    obj = ctypes.cast(this, ctypes.POINTER(ctypes.c_void_p))
    vtbl = ctypes.cast(obj[0], ctypes.POINTER(ctypes.c_void_p))
    return ctypes.WINFUNCTYPE(rest, ctypes.c_void_p, *atypes)(vtbl[idx])

class WasapiOut:
    """IAudioClient 独占渲染。write() 线程安全; 内部事件驱动线程喂设备。"""
    def __init__(self, rate):
        ole32 = ctypes.windll.ole32
        self._co = ole32.CoInitializeEx(None, 0x0)   # MTA
        enum = ctypes.c_void_p()
        hr = ole32.CoCreateInstance(ctypes.byref(_guid(_W_GUID['mmdev_enum_cls'])),
                                    None, 0x17,       # CLSCTX_ALL
                                    ctypes.byref(_guid(_W_GUID['mmdev_enum_iid'])),
                                    ctypes.byref(enum))
        if hr: raise RuntimeError('CoCreateInstance 0x%08X' % (hr & 0xFFFFFFFF))
        dev = ctypes.c_void_p()
        hr = _vt(enum, 4, ctypes.HRESULT, [ctypes.c_int, ctypes.c_int,
                ctypes.POINTER(ctypes.c_void_p)])(enum, 0, 0, ctypes.byref(dev))
        if hr: raise RuntimeError('GetDefaultAudioEndpoint 0x%08X' % (hr & 0xFFFFFFFF))
        self._dev = dev
        ac = ctypes.c_void_p()
        # Activate(REFIID, DWORD, PROPVARIANT*, void**)。
        # !! 栈位(第5参起)的指针型参数在本机 ctypes 3.12 WINFUNCTYPE 下传递损坏
        # !! (MultiByteToWideChar 对照实验实锤: 指针型栈位输出恒不写, 改整型即正常),
        # !! 故第 5 参 ppv 声明 c_ulonglong、传 addressof 纯地址绕开。
        iid_ac = _guid(_W_GUID['audio_client'])
        hr = _vt(dev, 6, ctypes.HRESULT,
                 [ctypes.c_void_p, ctypes.c_uint, ctypes.c_void_p,
                  ctypes.c_ulonglong])(
            dev, ctypes.byref(iid_ac), 0x17, 0, ctypes.addressof(ac))
        if hr: raise RuntimeError('Activate 0x%08X' % (hr & 0xFFFFFFFF))
        if not ac.value:
            # 本机 MMDevApi 实测: 个别机器 IMMDevice vtable 布局偏离公开 ABI
            # (GetId 落在 idx5 等), Activate 调用成功码但不出接口 —— 抛错回退 winmm
            raise RuntimeError('Activate 返回成功但未输出接口(该机音频栈异常)')
        # 独占+事件: 缓冲须为设备周期整数倍, 依次尝试 2/3/4 个默认周期
        self._ac = ac
        fmt = WAVEFORMATEX(1, OCH, rate, rate * OCH * 2, OCH * 2, 16, 0)
        per = ctypes.c_int64()                      # REFERENCE_TIME(100ns, 64位)
        _vt(ac, 8, ctypes.HRESULT, [ctypes.POINTER(ctypes.c_int64),
            ctypes.POINTER(ctypes.c_int64)])(ac, ctypes.byref(per), None)
        self.period_ms = per.value / 10000.0
        for mult in (2, 3, 4):
            dur = int(per.value) * mult
            # 独占模式 hnsPeriodicity 必须非 0 且 = hnsBufferDuration(MSDN);
            # 第 6 参(栈位 GUID*)同样用整型地址绕开 ctypes 栈位指针 bug
            r = _vt(ac, 3, ctypes.HRESULT, [ctypes.c_int, ctypes.c_uint,
                    ctypes.c_int64, ctypes.c_int64, ctypes.c_void_p,
                    ctypes.c_ulonglong])(ac, 1, 0x00040000,
                    dur, dur, ctypes.byref(fmt), 0)
            if r == 0:
                self.buf_mult = mult
                break
            self._init_hr = r
        else:
            raise RuntimeError('Initialize(exclusive) 0x%08X (设备被占或格式不支持)'
                               % (r & 0xFFFFFFFF))
        nf = ctypes.c_uint()
        _vt(ac, 4, ctypes.HRESULT, [ctypes.POINTER(ctypes.c_uint)])(
            ac, ctypes.byref(nf))
        self.frames = nf.value
        self._evt = ctypes.windll.kernel32.CreateEventW(None, 0, 0, None)
        _vt(ac, 12, ctypes.HRESULT, [ctypes.c_void_p])(ac, self._evt)
        rc = ctypes.c_void_p()
        _vt(ac, 13, ctypes.HRESULT, [ctypes.c_void_p,
                ctypes.POINTER(ctypes.c_void_p)])(
            ac, ctypes.byref(_guid(_W_GUID['audio_render'])), ctypes.byref(rc))
        self._rc = rc
        self._ring = bytearray()
        self._lk = threading.Lock()
        self._und = 0                    # 设备侧欠载帧数(诊断)
        self._run = True
        self._frm = rate * OCH * 2       # 每秒字节数
        _vt(ac, 9, ctypes.HRESULT, [])(ac)            # Start
        self._th = threading.Thread(target=self._render, daemon=True)
        self._th.start()

    def _render(self):
        kernel32 = ctypes.windll.kernel32
        rc, ac, evt = self._rc, self._ac, self._evt
        get_pad = _vt(ac, 6, ctypes.HRESULT, [ctypes.POINTER(ctypes.c_uint)])
        get_buf = _vt(rc, 3, ctypes.HRESULT, [ctypes.c_uint,
                     ctypes.POINTER(ctypes.POINTER(ctypes.c_ushort))])
        rel_buf = _vt(rc, 4, ctypes.HRESULT, [ctypes.c_uint, ctypes.c_uint])
        while self._run:
            kernel32.WaitForSingleObject(evt, 200)
            pad = ctypes.c_uint()
            if get_pad(ac, ctypes.byref(pad)) or self.frames <= pad.value:
                continue
            n = self.frames - pad.value
            p = ctypes.POINTER(ctypes.c_ushort)()
            if get_buf(rc, n, ctypes.byref(p)):
                continue
            need = n * OCH * 2
            with self._lk:
                take, self._ring = bytes(self._ring[:need]), self._ring[need:]
            if len(take) < need:
                self._und += (need - len(take)) // (OCH * 2)
                take += b'\x00' * (need - len(take))
            ctypes.memmove(p, take, need)
            rel_buf(rc, n, 0)

    # ---- player 后端接口 ----
    def write(self, b):
        with self._lk:
            if len(self._ring) > self._frm:      # 环上限 ~1s, 防失控
                del self._ring[:len(self._ring) - self._frm]
            self._ring += b

    def buffered_ms(self):
        with self._lk:
            return int(len(self._ring) * 1000.0 / self._frm)

    def reset(self):
        with self._lk:
            self._ring.clear()

    def close(self):
        self._run = False
        try:
            _vt(self._ac, 10, ctypes.HRESULT, [])(self._ac)      # Stop
        except Exception:
            pass

def open_wasapi(rate):
    """尝试 WASAPI 独占; 成功返回 WasapiOut, 失败抛异常(调用方回退 winmm)。"""
    if sys.platform != 'win32':
        raise RuntimeError('仅 Windows')
    return WasapiOut(rate)

def open_waveout(rate):
    if winmm is None:
        e = RuntimeError('winmm 不可用(非 Windows 或缺少多媒体组件)')
        e.mmerror = 6                      # 与"无音频输出设备"同档 → 仅统计模式
        raise e
    fmt = WAVEFORMATEX(1, OCH, rate, rate * OCH * 2, OCH * 2, 16, 0)
    hwo = ctypes.c_void_p()
    r = winmm.waveOutOpen(ctypes.byref(hwo), ctypes.c_uint(0xFFFFFFFF),
                          ctypes.byref(fmt), 0, 0, 0)
    if r:
        e = RuntimeError('waveOutOpen 失败 code=%d (采样率 %d 设备是否支持?)' % (r, rate))
        e.mmerror = r                      # 2=BADDEVICEID 3=NOTENABLED 6=NODRIVER: 无输出设备
        raise e
    return hwo

def player(hwo, q, rate, stop_event, log=print, buf=None):
    alive, pend, started = [], b'', False
    is_was = isinstance(hwo, WasapiOut)   # v5.5.0 WASAPI 独占后端: alive/WAVEHDR
                                          # 恒空, "在途"=hwo 内部 ring(事件驱动线程
                                          # 直喂设备), 各分支按 is_was 走对应接口
    avg_pkt = 0                       # 每包字节数(按首包估计, 用于领先量估算)
    last_trim, last_trim_log = 0.0, 0.0   # 软限追沿的执行/日志限速时刻
    dry, un_cnt, last_un_log = False, 0, 0.0    # 欠载(上升沿)/累计/日志限速
    cap_ms = MAX_INFLIGHT_MS          # 在途上限(欠载自适应加深至 300ms 封顶)
    need = rate * OCH * 2 * BLOCK_MS // 1000
    prebuf = rate * OCH * 2 * PREBUF_MS // 1000   # 按 audio 时长判定, 不受线程调度影响
    max_lead = rate * OCH * 2 * MAX_LEAD_MS // 1000
    while not stop_event.is_set():
        if buf is not None and buf.get('flush'):   # 接收侧流结束已 waveOutReset:
            buf['flush'] = False                   # 同步清残留/回未起播态(防重播旧音)
            pend = b''
            started = False
            dry = False
        # ① 先回收已播完的块(每轮都做): 后面的"在途上限"和欠载判断要用最新 inflight
        if alive:
            rest = []
            for blk, hdr in alive:
                if hdr.dwFlags & 1:           # WHDR_DONE
                    winmm.waveOutUnprepareHeader(hwo, ctypes.byref(hdr),
                                                 ctypes.sizeof(hdr))
                else:
                    rest.append((blk, hdr))
            alive = rest
        # ② 在途已达上限: **只等不取包**。
        #    旧实现是"先 q.get 取一包再 sleep(2ms)"——满载时每秒只能从队列取 500 包,
        #    而接收侧是 1000 包/s, 队列每秒多积 500 包, lead 持续爬升(实测开播 1s
        #    就到 182ms), 最后只能靠软限追沿硬丢音频(听感=咔哒/断音)。拦截提前到取包前。
        # winmm 在途=驱动队列(alive), WASAPI 在途=ring 未消费字节(buffered_ms)
        inflight_ms = hwo.buffered_ms() if is_was else len(alive) * BLOCK_MS
        if started and inflight_ms >= cap_ms:
            if buf is not None:
                buf['ms'] = int((len(pend) + q.qsize() * max(avg_pkt, 1)) * 1000.0
                                / (rate * OCH * 2) + inflight_ms)
            time.sleep(0.001)      # timeBeginPeriod(1) 已开, 实睡 ~1ms
            continue
        try:
            data = q.get(timeout=0.5)
        except queue_mod.Empty:
            continue
        if data is None:
            break
        # 包长做平滑跟踪(换台/变速时包长会变), 用于领先量估算
        avg_pkt = max(len(data), 1) if not avg_pkt else (avg_pkt * 7 + len(data)) // 8
        pend += data
        if not started and len(pend) < prebuf:
            continue
        if not started:
            # 起播跳直播沿(2026-11-13): 设备开启/首写期间接收侧已积压数百 ms,
            # 旧逻辑靠软限以 5~15ms/100ms 追 30s 才到稳态; 起播直接清空队列
            # (pend 已攒足 prebuf), 秒进低延迟稳态。代价=跳过开头一小段。
            bl_ms = q.qsize() * max(avg_pkt, 1) * 1000.0 / (rate * OCH * 2)
            if bl_ms > PREBUF_MS + 50:
                drop_n = 0
                while True:
                    try:
                        q.get_nowait()
                        drop_n += 1
                    except queue_mod.Empty:
                        break
                log('起播跳直播沿: 丢弃积压 %.0fms(%d 包)' % (bl_ms, drop_n))
        # 追直播沿: 硬重置只留给断流级积压; 常规突发(攒批/迟到)由动态软限分级消化
        lead = len(pend) + q.qsize() * avg_pkt
        lead_ms = lead * 1000.0 / (rate * OCH * 2)
        # 动态软限水位 = 在途上限(自适应) + 预攒 + 余量 —— 跟着缓冲一起涨,
        # 攒批周期内的水位波动落在此带内不再砍(攒批余波按秒级平滑排干)
        soft_ms = cap_ms + PREBUF_MS + SOFT_MARGIN_MS
        if started and lead > max_lead:
            drop_n = 0
            while True:
                try:
                    q.get_nowait()
                    drop_n += 1
                except queue_mod.Empty:
                    break
            pend = b''
            if is_was:
                hwo.reset()             # 清 ring 即清在途(_render 补静音过渡)
            else:
                winmm.waveOutReset(hwo)
                if alive:               # 必须逐块 unprepare, 否则驱动 header 泄漏
                    for blk, hdr in alive:   # (长会话越播越脏/卡顿的元凶)
                        winmm.waveOutUnprepareHeader(hwo, ctypes.byref(hdr),
                                                     ctypes.sizeof(hdr))
            alive = []
            started = False
            dry = False
            log('断流级积压 %.0fms, 丢弃 %d 包跳回直播沿' % (lead_ms, drop_n))
            continue
        elif started and lead_ms > soft_ms and len(pend) >= need \
                and time.time() - last_trim > 0.1:
            # 分级追沿: 超出软限越多步子越大(5/10/15ms), 每 100ms 一步 ——
            # 时钟漂移/攒批余波平滑排干, 不累积到硬重置
            excess = lead_ms - soft_ms
            step_ms = 5 if excess < 60 else (10 if excess < 150 else 15)
            cut = (rate * OCH * 2 * step_ms // 1000 // need) * need
            cut = max(cut, need)
            pend = pend[cut:]
            now2 = time.time()
            last_trim = now2
            if now2 - last_trim_log >= 1.0:
                log('软限追沿: lead %.0fms (软限 %.0fms), 丢 %dms' %
                    (lead_ms, soft_ms, cut * 1000 // (rate * OCH * 2)))
                last_trim_log = now2
        # 欠载检测(上升沿): 已起播但驱动队列与 pend 双空 = 断流实锤。
        # 对策: ①在途上限 +20ms 自适应加深(封顶 300ms, 覆盖 AP 攒批周期)——
        # 好网络不受影响, 差网络自动长到够用; ②回预攒水位再起播, 打断
        # "断流→裸奔恢复→立即再断流"的连锁欠载
        # WASAPI 欠载判据: ring 近空(<1 块); winmm: 驱动队列(alive)空
        if started and not dry and len(pend) < need and \
                (hwo.buffered_ms() < BLOCK_MS if is_was else not alive):
            dry = True
            un_cnt += 1
            if cap_ms < 300:
                cap_ms += 20
            started = False           # 回预攒再起播(仅多付一次 PREBUF 延迟)
            if time.time() - last_un_log >= 2.0:
                log('欠载 #%d, 在途上限自适应升至 %dms' % (un_cnt, cap_ms))
                last_un_log = time.time()
        # 上报缓冲深度(Python 侧积压 + 设备在途)与欠载计数, GUI 实时显示
        if buf is not None:
            buf['ms'] = int((len(pend) + q.qsize() * avg_pkt) * 1000.0 / (rate * OCH * 2)
                            + (hwo.buffered_ms() if is_was else len(alive) * BLOCK_MS))
            buf['un'] = un_cnt
        # 提交: pend 里攒够一块就写设备(在途上限已在循环开头拦截)
        while len(pend) >= need:
            blk, pend = pend[:need], pend[need:]
            if is_was:                        # 塞 ring, _render 线程事件驱动取走
                hwo.write(blk)
            else:
                hdr = WAVEHDR(lpData=blk, dwBufferLength=len(blk))
                winmm.waveOutPrepareHeader(hwo, ctypes.byref(hdr), ctypes.sizeof(hdr))
                winmm.waveOutWrite(hwo, ctypes.byref(hdr), ctypes.sizeof(hdr))
                alive.append((blk, hdr))      # blk 必须保活到播完
            started = True
            dry = False                       # 已重新供料, 解除欠载沿检测封锁
    # 退出前必须先摘掉设备上的所有缓冲, 再让 alive 被回收:
    # 否则本线程结束即 GC 释放 WAVEHDR/blk, 而 winmm 仍持有这些指针 ->
    # 驱动写已释放内存 = 堆损坏, 之后任意线程(如发包的 bytes 分配)都会
    # Windows 原生 AV 闪退。顺序: reset(异步归还) -> 逐块 unprepare -> 清引用。
    # WASAPI: close() 里 Stop 设备即可, ring 缓冲由 GC 回收(驱动不持指针)。
    try:
        if is_was:
            hwo.close()
        else:
            winmm.waveOutReset(hwo)
            for blk, hdr in alive:
                winmm.waveOutUnprepareHeader(hwo, ctypes.byref(hdr), ctypes.sizeof(hdr))
    except Exception:
        pass
    alive = []

# ---------------- 输出音量(只改播放增益, 不影响接收/统计) ----------------
VOL = 1.0          # 1.0 = 原始电平; GUI 滑块 0..200%

def apply_vol(b):
    """S16LE 立体声字节整体乘增益。audioop.mul 自带饱和截断(不会环绕失真);
    100% 时直通, 零开销。"""
    g = VOL
    if g == 1.0 or not b:
        return b
    if audioop is not None:
        try:
            return audioop.mul(b, 2, g)
        except Exception:
            return b
    import array                     # 兜底(Python 3.13+ 无 audioop): 逐样本饱和
    a = array.array('h')
    a.frombytes(b)
    for i, s in enumerate(a):
        v = int(s * g)
        a[i] = 32767 if v > 32767 else (-32768 if v < -32768 else v)
    return a.tobytes()

# ---------------- 载荷转换: S24_3LE / S32 交错 → S16 立体声(运行中可切换) ----------------
def conv(payload, step=1):
    """返回 S16LE 立体声字节。按 SEL 声道数路由: 1 个=复制双耳 / 2 个=分送 L/R /
    >=3 个=声像定位混音到立体声(ITU-R BS.775 风格)。

    算法(v5.2): 全程无逐帧 Python 循环 —— 交错流里取某声道 16bit 样本序列
    = 两条跨步切片(bytes[起:止:跨距] 为 C 级 memcpy, 跨距=帧长×step);
    输出立体声排布用 bytearray 跨步赋值一次铺成(0::4/1::4/2::4/3::4)。
    混音路径先逐声道切片+struct.unpack(C)成列, 再 zip 求和——把每样本算术
    从"帧×声道"两级循环降为单级。1000pps 下转换开销 ~数µs/包, 接收线程
    GIL 占用极小, 低延迟档的 player 调度更稳。
    step>1 时每 step 帧取 1 帧(输出降速, 监听用, 有混叠)。"""
    frame = max(1, NCH) * SBY               # NCH=0(清空声道)时板端仍按空选保护发 1 声道,
    n = len(payload) // frame               # 帧长按 1 声道估(否则 0 除零崩接收线程)
    n_out = (n + step - 1) // step          # 保留帧数(ceil, 兼容非整除抽取)
    stride = frame * step
    end = len(payload)
    chs = SEL                               # 一次取用整个元组, 与切换线程无撕裂
    nsel = len(chs)
    d = 1 if FMT_S24 else 2

    def _fit(b, n):
        """切片定长: 换台瞬间在途包布局(ch_count)与 NCH 短暂不一致时,
        跨步切片长度与槽位数不齐——补零/截断, 任何组合不再抛异常(≤1s 杂音换安全)"""
        return b[:n] if len(b) >= n else b + b'\x00' * (n - len(b))

    if not chs:
        return b'\x00' * (n_out * 4)        # 静音: 帧数×立体声4B
    out = bytearray(4 * n_out)
    if nsel == 2:
        # L=chs[0], R=chs[1]: 每声道"样本低/高字节"各一条跨步切片直接铺入
        o1 = chs[0] * SBY + d
        o2 = chs[1] * SBY + d
        out[0::4] = _fit(payload[o1:end:stride], n_out)
        out[1::4] = _fit(payload[o1 + 1:end:stride], n_out)
        out[2::4] = _fit(payload[o2:end:stride], n_out)
        out[3::4] = _fit(payload[o2 + 1:end:stride], n_out)
    elif nsel == 1:
        o1 = chs[0] * SBY + d
        b0 = _fit(payload[o1:end:stride], n_out)
        b1 = _fit(payload[o1 + 1:end:stride], n_out)
        out[0::4] = b0
        out[1::4] = b1
        out[2::4] = b0                      # 单声道复制到双耳
        out[3::4] = b1
    else:
        # >=3 声道: 7.1→立体声下混, 系数取 ITU-R BS.775 / ATSC A/52 常用值
        # (256 = 1.0): L/R=1.0, C=0.707(双耳), LFE=0.5(双耳), SL/SR/SBL/SBR=0.707。
        # 环绕不再按 1.0 直加 —— 旧系数在"多声道同相"时(例如测试素材把 L/R 复制到
        # 4 对声道)左耳会叠到 3 倍满幅, 硬 clamp 成方波正是沙沙声的来源。
        # 叠加后仍可能越界, 由下面每耳的峰值限幅器兜底。
        ROUTE = ((256, 0), (0, 256), (181, 181), (128, 128),      # L  R   C   LFE
                 (181, 0), (0, 181), (181, 0), (0, 181))          # SL SR  SBL SBR
        m = bytearray(2 * n_out)
        lcols, rcols, ms = [], [], []      # ms: numpy 路径的逐声道 m 快照
        for c in chs:
            o = c * SBY + d
            m[0::2] = _fit(payload[o:end:stride], n_out)
            m[1::2] = _fit(payload[o + 1:end:stride], n_out)
            wl, wr = ROUTE[SLOT_OF_POS[c] if SLOT_OF_POS and c < len(SLOT_OF_POS) else c]
            if _np is not None:
                ms.append((bytes(m), wl, wr))     # 快照(下轮循环覆写 m)
                continue
            col = struct.unpack('<%dh' % n_out, m)
            if wl:
                lcols.append(col if wl == 256 else [(v * wl) >> 8 for v in col])
            if wr:
                rcols.append(col if wr == 256 else [(v * wr) >> 8 for v in col])

        lsum_np = rsum_np = None
        if _np is not None and ms:
            # numpy 快路径(可选加速): 全部选中声道的 m 拼成一个矩阵, 一次 frombuffer,
            # (v*wl)>>8 加权后整列求和 —— 把每样本算术从 Python 循环压成寥寥几次
            # 整数组操作(8ch 混音实测 ~2x)。数值与纯 Python 逐位一致:
            # ① wl=256 也走 (v*256)>>8: v*256 低 8 位恒 0, >>8 精确还原(含负数);
            # ② numpy 对负数 >> 亦为算术右移; ③ int32 乘移求和全程整数精确无溢出
            # (|v*wl|<=32767*256, 8 列和 <= 2^26)。
            cols = _np.frombuffer(b''.join(mm for mm, _, _ in ms), dtype=_np.int16) \
                .reshape(len(ms), n_out)                       # (声道, 帧), int16×int32→int32
            w = _np.array([[wl for _, wl, _ in ms], [wr for _, _, wr in ms]],
                          dtype=_np.int32).reshape(2, len(ms), 1)
            if w[0].any():
                lsum_np = ((cols * w[0]) >> 8).sum(axis=0, dtype=_np.int32)
            if w[1].any():
                rsum_np = ((cols * w[1]) >> 8).sum(axis=0, dtype=_np.int32)

        def _pack(sums_np, cols, st):
            """一耳求和 → 峰值限幅 → int16 字节。
            限幅器: 越过满幅立即按峰值压低(不产生削波/方波), 之后每包 +0.2% 恢复
            (0.002×1000pps, -6dB 约 0.25s 回满, 量级 ~24dB/s), 正常电平完全不受影响、不改音色。
            numpy 路径: sums_np=已加权求和的 int32 列向量, cols=None;
            纯 Python 路径: cols=加权列 list, sums_np=None。
            st = LIM_STATE[0/1] = [当前增益, 限制事件数, 本周峰值保持]"""
            if sums_np is not None:
                peak = int(_np.abs(sums_np).max()) if len(sums_np) else 0
                sums = None
            else:
                sums = list(map(sum, zip(*cols)))          # C 级逐样本求和
                peak = max(max(sums), -min(sums))
            g = st[0]
            if peak * g > 32767:
                g = 32767.0 / peak                     # 立即压住峰值
                st[1] += 1
            elif g < 1.0:
                g = min(1.0, g + 0.002)                # 恢复(每包 +0.2%, 1000pps 下量级 ~24dB/s)
            st[0] = g
            pk_after = peak * g                        # 限幅后的实际输出峰值
            if pk_after > 32767:
                pk_after = 32767
            if pk_after > st[2]:
                st[2] = pk_after                       # 峰值保持(供 UI 显示 dBFS)
            if g >= 1.0:                               # 此路峰值必在 int16 范围内
                return (sums_np.astype('<i2').tobytes() if sums_np is not None else
                        struct.pack('<%dh' % n_out, *sums))
            if sums_np is not None:
                # clip 后向零截断: 与 max(-32768, min(32767, int(s*g))) 逐位一致
                return _np.clip(sums_np * g, -32768, 32767).astype('<i2').tobytes()
            return struct.pack('<%dh' % n_out,
                               *[max(-32768, min(32767, int(s * g))) for s in sums])

        if lsum_np is not None or lcols:    # 仅选单侧声道时另一耳保持静音
            lb = _pack(lsum_np, lcols, LIM_STATE[0])
            out[0::4] = lb[0::2]
            out[1::4] = lb[1::2]
        if rsum_np is not None or rcols:
            rb = _pack(rsum_np, rcols, LIM_STATE[1])
            out[2::4] = rb[0::2]
            out[3::4] = rb[1::2]
    return bytes(out)

# ---------------- 接收引擎(GUI/CLI 共用) ----------------
def run_receiver(cfg, stop_event, log=print, stats=None):
    """cfg: dict(port, group, rate, chs, fmt, raw, profile); 阻塞运行至 stop_event。"""
    global PREBUF_MS, MAX_INFLIGHT_MS, SOFT_MARGIN_MS, OD
    profile = cfg.get('profile') or 'low'
    PREBUF_MS, MAX_INFLIGHT_MS = PROFILES[profile]   # player 线程启动前生效
    SOFT_MARGIN_MS = {'ultra': 20, 'low': 50, 'normal': 70, 'stable': 80}[profile]
    od = bool(cfg.get('od'))                          # v1.1 按需单播
    if od:
        if cfg['raw']:
            log('错误: 按需单播模式不支持裸流')
            return
        if not (cfg.get('iface') or ''):
            log('错误: 按需单播模式必须填本机网卡 IP(发现包自报 IP, 板按它回发流)')
            return
        try:
            zone = int(cfg.get('zone') or 0)
            assert 0 <= zone <= 255
        except (TypeError, ValueError, AssertionError):
            log('错误: 区域ID 须为 0~255')
            return
        cfg = dict(cfg, fmt='s32', rate=48000)        # 板端 ondemand 载荷恒 s32/48k
    OD = od
    apply_cfg(cfg['port'], cfg['rate'], cfg['chs'], cfg['fmt'] == 's24')
    raw = cfg['raw']
    iface = cfg.get('iface') or ''

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)   # 快速重启时旧 socket 未释放也能重绑
    want_rcvbuf = 4 * 1024 * 1024
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, want_rcvbuf)
    got_rcvbuf = sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
    if got_rcvbuf < want_rcvbuf:           # Linux 默认 rmem_max(~208KB)会静默钳制, 必须点破
        log('警告: 接收缓冲申请 %dKB 仅生效 %dKB(内核钳制, 攒批突发易丢包); '
            'Linux 可提权执行 sysctl -w net.core.rmem_max=4194304 后重开'
            % (want_rcvbuf // 1024, got_rcvbuf // 1024))
    sock.settimeout(1.0)                   # 周期唤醒: 流结束检测/停止响应
    # 多网卡(Windows 上常见 VPN/VMnet): 绑定并 join 到指定网卡, 避免组播进错口
    sock.bind((iface or '0.0.0.0', PORT))
    buf = {'ms': 0, 'un': 0, 'flush': False}   # player 回写: 水位/欠载计数/流结束清除标记
    if od:
        log('按需单播模式: %s:%d 收单播子集流, %dHz 监听 %s' %
            (iface, PORT, RATE, sel_desc()))
        threading.Thread(target=beacon, args=(stop_event, iface, log, buf, zone),
                         daemon=True).start()
    elif cfg['group'] not in ('', 'none'):
        mreq = socket.inet_aton(cfg['group']) + socket.inet_aton(iface or '0.0.0.0')
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        log('监听 %s:%d 组播 %s, %dHz 监听 %s, %s%s' %
            (iface or '0.0.0.0', PORT, cfg['group'], RATE, sel_desc(), cfg['fmt'],
             ', 裸流' if raw else ''))
    else:
        log('监听 %s:%d (不 join, 广播/单播), %dHz 监听 %s, %s%s' %
            (iface or '0.0.0.0', PORT, RATE, sel_desc(), cfg['fmt'],
             ', 裸流' if raw else ''))
    log('等待板端发送... (板端示例: adb shell "/tmp/arsend --compat esp32 --bind <eth1 IP>")')

    # 音频输出: WASAPI 独占优先(v5.5.0, 绕过系统混音引擎省 30~50ms) → 失败回退
    # winmm 共享; 再无可用输出设备 → 仅统计模式; 采样率不支持时自动降 48k
    orate, step = RATE, 1
    hwo = None
    was_mode = cfg.get('wasapi', 'auto')
    q = queue_mod.Queue()
    pth = None
    if was_mode != 'off':
        try:
            hwo = open_wasapi(orate)
            log('WASAPI 独占输出: 设备周期 %.1fms × %d 缓冲(绕过混音引擎, '
                '端到端比共享模式低 ~30-50ms)' % (hwo.period_ms, hwo.buf_mult))
        except Exception as e:
            hwo = None
            if was_mode == 'on':
                sock.close()
                raise RuntimeError('WASAPI 独占失败: %s (设备被占? --wasapi off 回退 winmm)' % e)
            log('WASAPI 独占不可用(%s) → 回退 winmm 共享模式' % e)
    if hwo is None:
        try:
            hwo = open_waveout(orate)
        except RuntimeError as e:
            if getattr(e, 'mmerror', 0) in (2, 3, 6):    # 无音频输出设备, 不影响统计
                log('无音频输出设备 → 仅统计模式(收包/到达率, 不播放)')
            elif RATE % 48000:
                sock.close()
                raise                           # 非 48k 整数倍, 无法降速
            else:
                orate, step = 48000, RATE // 48000
                log('waveOut 不支持 %dHz → 输出降为 48000（每 %d 帧取 1, 监听可用有混叠）'
                    % (RATE, step))
                hwo = open_waveout(orate)
    if hwo is not None:
        is_was = isinstance(hwo, WasapiOut)
        if not is_was:
            winmm.timeBeginPeriod(1)  # 1ms 系统时钟粒度: 否则 sleep 实睡 ~15.6ms, 低在途必断续
        sys.setswitchinterval(0.001)  # GIL 5ms→1ms: 接收线程持锁时 player 提交排队更短
        pth = threading.Thread(target=player, args=(hwo, q, orate, stop_event, log, buf),
                               daemon=True)
        pth.start()
        if is_was:
            log('延迟档位 %s: 起播预攒 %dms + ring 在途上限 %dms (WASAPI 独占, 无系统混音)'
                % (profile, PREBUF_MS, MAX_INFLIGHT_MS))
        else:
            log('延迟档位 %s: 起播预攒 %dms + 在途上限 %dms (块粒度 %dms, 另有系统混音引擎 ~30-50ms)'
                % (profile, PREBUF_MS, MAX_INFLIGHT_MS, BLOCK_MS))

    t0 = None
    expect_seq = None
    rx = fec = loss = late = 0
    rxb = 0                            # v1.3: PT=100 批量流包计数(b4=)
    batch_logged = False
    # NTP 端到端(2026-11-13): e2e = (本机now + ntp_offset) - ts/48000
    #   到达口径 = 发端采样时刻 -> PC 收到; 播放口径 ≈ 到达 + 缓冲水位 buf.ms
    #   ts 32->64 位扩展: 差值跳变 >2^31 视为回绕/发端重锚, 高位重算
    ts_ext = None
    e2e_n = 0; e2e_sum = 0.0; e2e_min = 1e9; e2e_max = -1e9; e2e_last = None
    # 录制: 收到的数据包载荷按线上格式原样落盘(1MB 缓冲, ~每 650 包一次写盘)。
    # 断洞补零(与播放补静音同 200ms 上限)保时间轴; 迟到包在写入点之前已丢弃。
    rec_f = open(cfg['record'], 'wb', buffering=1 << 20) if cfg.get('record') else None
    if rec_f is not None:
        log('录制: 载荷落盘 %s(按线上格式: 帧长/声道数=注册的 --ch 子集)' % cfg['record'])
    last_bucket = -1
    silence16 = None
    last_pkt = 0.0
    flushed = True
    consec_late = 0
    pk_probe = 0                       # 仅统计模式下的峰值抽样计数

    def report(final=False):
        tot = rx + loss
        pct = 100.0 * rx / tot if tot else 0.0
        # 限幅事件 + 本窗口峰值(转 dBFS): 只在 >=3 声道混音路径有意义(1~2 声道是
        # 直通/复制, 不求和也不会越界)。峰值贴到 -0.1dBFS 且限幅数在涨 = 素材本身
        # 已满幅叠加, 属于正常保护; 长时间限幅说明所选声道组合电平过高。
        lim = LIM_STATE[0][1] + LIM_STATE[1][1]
        if len(REQ) >= 3:
            peak = max(LIM_STATE[0][2], LIM_STATE[1][2])
            LIM_STATE[0][2] = LIM_STATE[1][2] = 0
            peak_db = (20.0 * math.log10(peak / 32768.0)) if peak > 0 else -120.0
        else:
            peak_db = None
        if stats:
            stats(dict(rx=rx, fec=fec, late=late, loss=loss, pct=pct, rxb=rxb,
                       ntp=buf.get('ntp_ms'),
                       rtt=buf.get('ntp_rtt_ms'),
                       e2e=(e2e_sum / e2e_n) if e2e_n else None,
                       e2e_last=e2e_last,
                       buf=buf['ms'], un=buf.get('un', 0), lim=lim, peak=peak_db,
                       t=(time.time() - t0) if t0 else 0.0, final=final, raw=raw))
        elif raw:
            # 裸流无 RTP 头/无 seq: 丢失/到达率无从计算 —— 只报收包计数,
            # 不显示 loss=0/100% 的假统计
            log('%s rx=%d(无seq,仅计数) un=%d%s%s' %
                ('结束' if final else 't=%.0fs' % (time.time() - t0), rx,
                 buf.get('un', 0),
                 (' 峰值=%.1fdBFS' % peak_db) if peak_db is not None else '',
                 (' 限幅=%d' % lim) if lim else ''))
        else:
            extra = ' fec=%d late=%d%s%s%s' % (
                fec, late, ' b4=%d' % rxb if rxb else '',
                (' ntp=%+.1fms' % buf['ntp_ms']) if buf.get('ntp_ms') is not None else '',
                (' e2e=%.0f/%.0f/%.0fms(+buf%.0f=播)' %
                 (e2e_sum / e2e_n, e2e_min, e2e_max, buf.get('ms') or 0))
                if e2e_n else '')
            log('%s rx=%d%s loss=%d un=%d 到达率=%.1f%%%s%s' %
                ('结束' if final else 't=%.0fs' % (time.time() - t0), rx, extra,
                 loss, buf.get('un', 0), pct,
                 (' 峰值=%.1fdBFS' % peak_db) if peak_db is not None else '',
                 (' 限幅=%d' % lim) if lim else ''))

    try:
        while not stop_event.is_set():
            try:
                data, addr = sock.recvfrom(65535)
            except socket.timeout:
                # 流结束检测: >2s 无包 → 丢弃残留缓冲立即收声(否则拖尾)
                if last_pkt and not flushed and time.time() - last_pkt > 2.0:
                    while not q.empty():
                        try:
                            q.get_nowait()
                        except queue_mod.Empty:
                            break
                    if hwo is not None:
                        if isinstance(hwo, WasapiOut):
                            hwo.reset()
                        else:
                            winmm.waveOutReset(hwo)
                    buf['flush'] = True       # 通知 player 清残留/回未起播态
                    flushed = True
                    expect_seq = None          # 下次启动按新流重新同步
                    consec_late = 0
                    log('流结束(2s 无包), 缓冲已清' if REQ
                        else '已静音(未选声道), 板端停发; 选回声道即自动复流')
                    report()                   # 打一份收尾快照(含峰值/限幅), 便于事后备查
                continue
            now = time.time()
            if t0 is None:
                t0 = now
                log('首包来自 %s:%d' % (addr[0], addr[1]))   # 区分组播/单播来源
                if not raw and cfg.get('ntp', True):
                    # 流源 IP = 发端 = ts 时基: 后台周期对时(诊断用, 播放不受影响)
                    threading.Thread(target=_ntp_thread,
                                     args=(addr[0], stop_event, log, buf),
                                     daemon=True).start()
            if raw:
                payload = data
            else:
                if len(data) < 13 or (data[0] >> 6) != 2:
                    continue
                pt = data[1] & 0x7f
                if pt in (97, 101):            # FEC: 97=1ms 流 / 101=v1.3 批量流, 只计数
                    fec += 1
                    continue
                if pt not in (96, 100):        # 媒体: 96=1ms / 100=v1.3 批量; 其余丢弃
                    continue
                if pt == 100:
                    rxb += 1
                    if not batch_logged:
                        batch_logged = True
                        frames = len(data) - 12
                        per_ms = 48 * max(1, len(REQ)) * SBY
                        log('v1.3 批量流(PT=100): 载荷 %dB ≈ %dms/包(订阅 %d 声道)'
                            % (frames, max(1, frames // max(1, per_ms)),
                               max(1, len(REQ))))
                seq = (data[2] << 8) | data[3]
                payload = data[12:]
                # NTP 端到端(到达口径): ts 32->64 扩展 + 与发端时基对齐后计算
                # 模差法回绕天然正确; |ts_step|>1s(48000) = 发端重锚 -> 重置会话基准
                # (ts_step 勿名 step: 下文 conv(payload, step) 的 step 是样本字节步长)
                ts32 = struct.unpack('!I', data[4:8])[0]
                if ts_ext is None:
                    ts_ext = ts32
                else:
                    delta = (ts32 - (ts_ext & 0xFFFFFFFF)) & 0xFFFFFFFF
                    ts_step = delta if delta < 0x80000000 else delta - 0x100000000
                    if abs(ts_step) > 48000:
                        ts_ext = ts32
                        e2e_n = 0; e2e_sum = 0.0; e2e_min = 1e9; e2e_max = -1e9
                    else:
                        ts_ext += ts_step
                off_ms = buf.get('ntp_ms')
                if off_ms is not None:
                    e2e = (now + off_ms / 1000.0 - ts_ext / 48000.0) * 1000.0
                    e2e_last = e2e
                    e2e_sum += e2e; e2e_n += 1
                    if e2e < e2e_min: e2e_min = e2e
                    if e2e > e2e_max: e2e_max = e2e

            if hwo is not None:              # 播放: 每包转换并入队
                s16 = apply_vol(conv(payload, step))
                silence16 = b'\x00' * len(s16)   # 每包刷新基准长: 流格式(声道数/抽取)
                                                    # 运行中变化后, 补静音块不致长度过期
            elif not raw:
                # 仅统计模式: 每 10 包抽样跑一次 conv(只为峰值/限幅统计),
                # 无输出设备时也能看电平, 开销 ≈ 播放路径的 1/10
                pk_probe += 1
                if pk_probe % 10 == 0:
                    conv(payload, step)

            if not raw:
                if expect_seq is not None:
                    gap = (seq - expect_seq) & 0xFFFF
                    if gap and gap < 0x8000:     # 断洞 = 丢包, 补等长静音保时间轴
                        loss += gap
                        if hwo is not None:
                            # 补静音上限 200ms: 一次断洞上千包(发送端重启/长时间
                            # 卡顿)时若全补, 播放水位会被顶高且永远追不回(延迟暴涨);
                            # 只补前 200ms, 其余靠 seq 重锚自然接上(听感=短静音后继续)
                            pkt_ms = ((len(s16) // 4) * 1000.0 / orate) if s16 else 1.0
                            max_pk = max(1, int(MAX_SILENCE_MS / max(pkt_ms, 0.05)))
                            q.put(silence16 * min(gap, max_pk))
                        if rec_f is not None:
                            # 录制补零同 200ms 上限(约 200 包@1ms/包): 保时间轴连续,
                            # 又不因发送端重启的巨洞写出几十 MB 空洞
                            rec_f.write(b'\x00' * (len(payload) * min(gap, 200)))
                    elif gap >= 0x8000:          # 迟到/乱序包: 该位置已补静音, 丢弃
                        late += 1
                        consec_late += 1
                        if consec_late >= 200:   # 连续大量迟到=发送端重启, 重新同步基线
                            expect_seq = seq
                            consec_late = 0
                            log('检测到流重启, 已重新同步')
                        continue
                expect_seq = (seq + 1) & 0xFFFF
                consec_late = 0
            rx += 1
            last_pkt = now
            if rec_f is not None:
                rec_f.write(payload)
            if od:
                buf['last_pkt'] = now
            flushed = False
            if hwo is not None:
                q.put(s16)

            bucket = int((now - t0) // 5)
            if bucket != last_bucket:
                last_bucket = bucket
                report()
    except KeyboardInterrupt:
        pass
    finally:
        report(final=True)
        if rec_f is not None:
            rec_f.close()
        sock.close()
        if hwo is not None:
            q.put(None)
            pth.join(timeout=2.0)
            if pth.is_alive():
                # 播放线程未退出(极罕见阻塞): 跳过设备关闭——宁可泄漏一个句柄
                # 也不 close 正被 waveOutWrite 使用的句柄(winmm 原生崩溃元凶)
                log('警告: 播放线程未及时退出, 跳过设备关闭(防竞态崩溃)')
            else:
                if isinstance(hwo, WasapiOut):
                    hwo.close()
                else:
                    winmm.waveOutReset(hwo)
                    winmm.waveOutClose(hwo)
                    winmm.timeEndPeriod(1)
        log('接收已停止')

# ---------------- CLI ----------------
def cli_main():
    ap = argparse.ArgumentParser(description='audio_router UDP 音频接收播放器')
    ap.add_argument('--port', type=int, default=5004)
    ap.add_argument('--group', default='239.100.7.1',
                    help="组播组(契约默认 239.100.7.1); 'none'=不加入(收广播/单播)")
    ap.add_argument('--rate', type=int, default=48000,
                    help='流采样率, 必须与发送端一致! 48000/96000(兼容档)/192000(定案档)')
    ap.add_argument('--ch', default='0,1',
                    help='要监听的源声道, 1~8 个: 1 个=复制双耳, 2 个=分送 L/R, '
                         '>=3 个=混音双耳; 如 0,1 或 4 或 0,1,4,5,6,7; GUI 运行中可随时切换')
    ap.add_argument('--fmt', choices=['s24', 's32'], default='s24',
                    help='载荷样本格式: s24=S24_3LE 3字节(默认, esp32 兼容档) '
                         's32=4字节槽(现役 48k 定案档; 按需单播模式恒为 s32, 无需指定)')
    ap.add_argument('--raw', action='store_true', help='裸流(无 RTP 头, 无 FEC/断洞处理)')
    ap.add_argument('--no-ntp', action='store_true',
                    help='关闭 NTP 对时探测(默认: 首包后向流源 :123 周期对时, '
                         '验证发端 ntpd 与方案A 精度, 统计行显示 ntp=offset)')
    ap.add_argument('--profile', choices=['ultra', 'low', 'normal', 'stable'], default='low',
                    help='延迟档位(预攒+在途ms): ultra=10+20(有线台架) / low=20+40 / '
                         'normal=30+60 / stable=60+150; 断续(网络抖动>缓冲)时升档')
    ap.add_argument('--wasapi', choices=['auto', 'on', 'off'], default='auto',
                    help='WASAPI 独占输出(绕过系统混音引擎, 端到端省 ~30-50ms): '
                         'auto=优先独占失败回退 winmm(默认) / on=强制(失败报错) / '
                         'off=不用(行为同 v5.4)。独占期间其他程序无声, 停止后恢复')
    ap.add_argument('--iface', default='',
                    help='本机网卡 IP(多网卡必填! 如 192.168.8.116), 绑定并 join 到该网卡')
    ap.add_argument('--od', action='store_true',
                    help='v1.1 按需单播: 广播发现包(:6000, slot_bit=--ch 选择), '
                         '收板端单播子集流; 须 --iface 本机IP')
    ap.add_argument('--zone', type=int, default=0,
                    help='区域 ID(0~255): 与板端 --zone 一致才被服务(多板共存隔离)')
    ap.add_argument('--record', default='',
                    help='录制: 收到的数据包载荷按线上格式原样落盘该文件'
                         '(8ch 全量录制用 --ch 0,1,2,3,4,5,6,7; 断洞补零≤200ms)')
    ap.add_argument('--gui', action='store_true', help='启动图形界面')
    args = ap.parse_args()

    if args.gui:
        gui_main()
        return
    chs = [int(x) for x in args.ch.split(',') if x != '']
    if not (1 <= len(chs) <= 8) or any(c < 0 or c > 7 for c in chs) or len(set(chs)) != len(chs):
        ap.error('--ch 需要 1~8 个 0..7 的不同声道')
    if args.od and not args.iface:
        cand = local_ipv4s()
        if cand:
            args.iface = cand[0][0]
            print('按需单播: 自动选择网卡 IP %s (可用: %s)' %
                  (args.iface, ', '.join(ip for ip, _ in cand)))
    cfg = dict(port=args.port, group=args.group, rate=args.rate, chs=chs,
               fmt=args.fmt, raw=args.raw, iface=args.iface, profile=args.profile,
               od=args.od, zone=args.zone, record=args.record,
               ntp=not args.no_ntp, wasapi=args.wasapi)
    stop = threading.Event()
    try:
        run_receiver(cfg, stop)
    except KeyboardInterrupt:
        stop.set()

# ---------------- GUI (tkinter) ----------------
CH_CHOICES = [('0,1 (L/R 立体声)', '0,1'), ('2,3 (C/LFE)', '2,3'),
              ('4,5 (SL/SR)', '4,5'), ('6,7 (SBL/SBR)', '6,7'),
              ('0,1,2,3 (前四混音)', '0,1,2,3'),
              ('0,1,2,3,4,5,6,7 (全八混音)', '0,1,2,3,4,5,6,7'),
              ('0 (L 单声道)', '0'), ('1 (R 单声道)', '1'),
              ('2 (C 单声道)', '2'), ('3 (LFE 单声道)', '3'),
              ('4 (SL 单声道)', '4'), ('5 (SR 单声道)', '5'),
              ('6 (SBL 单声道)', '6'), ('7 (SBR 单声道)', '7')]

def gui_main():
    import tkinter as tk
    from tkinter import ttk, scrolledtext

    # 高 DPI 适配: 125%/150% 缩放下不让系统位图拉伸(控件变糊、布局被顶开); 失败无副作用
    try:
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass

    root = tk.Tk()
    root.title('rxplay v5.3.0 - audio_router UDP 音频接收播放器')
    # 窗口尺寸不写死: 构建完控件后按"实际字号需求"自适应(见文件末尾 _autosize),
    # 高 DPI(125%/150%)下字体变大, 固定像素会让内容被挤/截断
    try:
        root.tk.call('tk', 'scaling', root.winfo_fpixels('1i') / 72.0)
    except Exception:
        pass
    style = ttk.Style()
    if 'clam' in style.theme_names():
        style.theme_use('clam')
    style.configure('Sel.TButton', background='#1565c0', foreground='white')

    stop_holder = [threading.Event()]   # 每次开始新建: 防 clear() 复活旧线程(崩溃根因)
    worker = [None]
    running = [False]                   # 是否处于运行态(停止按键仅在运行态发 tx_enable=0)
    ui_q = queue_mod.Queue()          # (kind, payload): log / stats

    frm = ttk.LabelFrame(root, text=' 接收参数（开始前设置；采样率/格式必须与发送端一致！）')
    frm.pack(fill='x', padx=10, pady=(8, 4))

    # 两行各自独立网格: 列宽互不牵扯, 标签与控件紧贴(此前共享列被另一行
    # 的宽控件顶开, 造成"延迟档位/端口/网卡IP"标签与控件间距过大)
    rowA = ttk.Frame(frm)
    rowA.pack(fill='x', padx=2, pady=(4, 2))
    ttk.Label(rowA, text='采样率:').grid(row=0, column=0, sticky='w', padx=4)
    rate_var = tk.StringVar(value='48000')
    rate_cb = ttk.Combobox(rowA, textvariable=rate_var, width=8, state='readonly',
                           values=('48000', '96000', '192000'))
    rate_cb.grid(row=0, column=1, padx=(0, 10))
    ttk.Label(rowA, text='格式:').grid(row=0, column=2, sticky='w', padx=4)
    fmt_var = tk.StringVar(value='s32')          # 现役板端为 s32 定案档
    ttk.Combobox(rowA, textvariable=fmt_var, width=6, state='readonly',
                 values=('s32', 's24')).grid(row=0, column=3, padx=(0, 10))
    ttk.Label(rowA, text='播放声道:').grid(row=0, column=4, sticky='w', padx=4)
    ch_var = tk.StringVar(value=CH_CHOICES[0][0])
    ch_cb = ttk.Combobox(rowA, textvariable=ch_var, width=16, state='readonly',
                         values=[c[0] for c in CH_CHOICES])
    ch_cb.grid(row=0, column=5, padx=(0, 10))
    ttk.Label(rowA, text='延迟档位:').grid(row=0, column=6, sticky='w', padx=4)
    lat_var = tk.StringVar(value='低')
    lat_cb = ttk.Combobox(rowA, textvariable=lat_var, width=6, state='readonly',
                          values=('极限', '低', '正常', '稳'))
    lat_cb.grid(row=0, column=7, padx=(0, 10))
    raw_var = tk.BooleanVar(value=False)
    ttk.Checkbutton(rowA, text='裸流', variable=raw_var).grid(
        row=0, column=8, sticky='w', padx=4)
    od_var = tk.BooleanVar(value=True)     # 默认按需单播(v1.1 现役形态), 不组播
    od_ck = ttk.Checkbutton(rowA, text='按需单播', variable=od_var)
    od_ck.grid(row=0, column=9, sticky='w', padx=4)
    rowA.columnconfigure(10, weight=1)     # 尾列留白吸收窗口拉伸

    rowB = ttk.Frame(frm)
    rowB.pack(fill='x', padx=2, pady=(2, 4))
    ttk.Label(rowB, text='组播组:').grid(row=0, column=0, sticky='w', padx=4)
    group_var = tk.StringVar(value='239.100.7.1')
    ttk.Entry(rowB, textvariable=group_var, width=16).grid(row=0, column=1, padx=(0, 10))
    ttk.Label(rowB, text='端口:').grid(row=0, column=2, sticky='w', padx=4)
    port_var = tk.StringVar(value='5004')
    ttk.Entry(rowB, textvariable=port_var, width=8).grid(row=0, column=3, padx=(0, 10))
    ttk.Label(rowB, text='网卡IP:').grid(row=0, column=4, sticky='w', padx=4)
    iface_var = tk.StringVar(value='')

    def iface_choices():
        """下拉项 "ip  (网卡名)"; 默认出口排最前, 多网卡可在此挑选。
        也可直接手填任意 IP(下拉是 state=normal 可编辑)。"""
        return ['%s  (%s)' % (ip, nm) if nm else ip for ip, nm in local_ipv4s()]

    def iface_ip():
        """从下拉文本/手填内容里取 IP: 取第一个空白分隔字段。"""
        s = iface_var.get().strip().split()
        return s[0] if s else ''

    iface_cb = ttk.Combobox(rowB, textvariable=iface_var, width=24, state='normal',
                            values=iface_choices())
    iface_cb.grid(row=0, column=5, sticky='w', padx=(0, 2))
    if not iface_var.get():
        _v = list(iface_cb['values']) if iface_cb['values'] else []
        if _v:
            iface_var.set(_v[0])          # 默认选中默认出口网卡

    def refresh_iface():
        vals = iface_choices()
        iface_cb.configure(values=vals)
        cur = iface_ip()
        if vals and cur not in [v.split()[0] for v in vals]:
            iface_var.set(vals[0])
        do_log('本机网卡: %s' % (', '.join(vals) if vals else '(未探测到, 请手填 IP)'))

    ttk.Button(rowB, text='↻', width=3, command=refresh_iface).grid(
        row=0, column=6, sticky='w', padx=(0, 10))
    ttk.Label(rowB, text='区域ID:').grid(row=0, column=7, sticky='w', padx=4)
    zone_var = tk.StringVar(value='0')
    zone_entry = ttk.Entry(rowB, textvariable=zone_var, width=4)
    zone_entry.grid(row=0, column=8, sticky='w', padx=(0, 4))
    rowB.columnconfigure(9, weight=1)

    # 参数记忆: 开始/退出时落盘, 启动时恢复; 文件缺失、损坏、取值越界一律忽略,
    # 绝不因配置问题挡住启动(现场最怕"点了没反应")。
    CFG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rxplay_cfg.json')
    saved_geom = [None]        # 上次退出时的窗口尺寸(WxH), 有则恢复; 无则按内容自适应

    def load_cfg():
        try:
            with open(CFG_PATH, encoding='utf-8') as f:
                c = json.load(f)
        except Exception:
            return
        try:
            if c.get('rate') in ('48000', '96000', '192000'):
                rate_var.set(c['rate'])
            if c.get('fmt') in ('s24', 's32'):
                fmt_var.set(c['fmt'])
            if c.get('ch') in dict(CH_CHOICES):
                ch_var.set(c['ch'])
            if c.get('lat') in ('低', '正常', '稳'):
                lat_var.set(c['lat'])
            if isinstance(c.get('group'), str):
                group_var.set(c['group'])
            if isinstance(c.get('port'), str) and c['port'].isdigit():
                port_var.set(c['port'])
            if isinstance(c.get('iface'), str):
                iface_var.set(c['iface'])
            if isinstance(c.get('zone'), str) and c['zone'].isdigit() \
                    and int(c['zone']) <= 255:
                zone_var.set(c['zone'])
            raw_var.set(bool(c.get('raw', False)))
            od_var.set(bool(c.get('od', True)))
            if isinstance(c.get('geom'), str) and re.match(r'^\d+x\d+', c['geom']):
                saved_geom[0] = c['geom']
        except Exception:
            pass

    def save_cfg():
        try:
            with open(CFG_PATH, 'w', encoding='utf-8') as f:
                json.dump(dict(rate=rate_var.get(), fmt=fmt_var.get(), ch=ch_var.get(),
                               lat=lat_var.get(), group=group_var.get(),
                               port=port_var.get(), iface=iface_ip(),
                               zone=zone_var.get(), raw=raw_var.get(),
                               od=od_var.get(), geom=root.geometry()),
                          f, ensure_ascii=False, indent=1)
        except Exception:
            pass

    load_cfg()          # 恢复上次参数(首次运行无文件 = 保持默认)

    # 控制栏(上移到参数区下, 免得被日志隔在最底部): 状态灯 + 开始/停止
    btn_row = ttk.Frame(root)
    btn_row.pack(fill='x', padx=10, pady=(2, 4))
    state_lbl = tk.Label(btn_row, text='● 未运行', foreground='gray',
                         font=('Microsoft YaHei UI', 10, 'bold'))
    state_lbl.pack(side='left', padx=(0, 12))
    start_btn = ttk.Button(btn_row, text='▶ 开始接收', width=14)
    stop_btn = ttk.Button(btn_row, text='■ 停止', width=10, state='disabled')
    start_btn.pack(side='left', padx=4)
    stop_btn.pack(side='left', padx=4)

    # 音量滑块: 100% = 原始电平, 0..200% 可调(>100% 用于源电平偏小时补偿)
    def on_vol(v):
        global VOL
        VOL = max(0.0, float(v) / 100.0)
        vol_lbl.configure(text='音量 %d%%' % int(round(float(v))))
    vol_lbl = tk.Label(btn_row, text='音量 100%', font=('Microsoft YaHei UI', 9))
    vol_lbl.pack(side='right', padx=(6, 8))
    vol_scale = ttk.Scale(btn_row, from_=0, to=200, length=130, command=on_vol)
    vol_scale.set(100)                 # 触发一次 on_vol, 同步标签
    vol_scale.pack(side='right')

    # 运行中监听切换行: 输出恒立体声, 原子切换无需停流/重开 waveOut
    swf = ttk.LabelFrame(root, text=' 监听声道切换（多选混音: 点击切换选中, 1个=双耳 2个=分L/R ≥3个=混音）')
    swf.pack(fill='x', padx=8, pady=4)
    sel_lbl = tk.StringVar(value='未运行')
    sel_btns = []                            # (button, chs元组): 供选中态刷新

    def refresh_sel_buttons(clear=False):
        """选中声道涉及的按钮高亮(Sel.TButton, 按钮声道全在选中集合), 其余还原"""
        cur = set(REQ)
        for b, chs in sel_btns:
            sel = not clear and chs and set(chs) <= cur
            b.configure(style='Sel.TButton' if sel else 'TButton')

    def on_sel_btn(chs):
        """toggle: 按钮声道已全在选中集合 → 移除; 否则加入。可任意多选混音"""
        cur = set(REQ)
        if set(chs) <= cur:
            cur -= set(chs)
        else:
            cur |= set(chs)
        set_sel(sorted(cur))
        sel_lbl.set('当前: ' + sel_desc())
        do_log('监听切换 → ' + sel_desc())
        refresh_sel_buttons()

    def on_sel_quick(sel):
        set_sel(sel)
        sel_lbl.set('当前: ' + sel_desc())
        do_log('监听切换 → ' + sel_desc())
        refresh_sel_buttons()

    quick = ttk.Frame(swf)
    quick.pack(fill='x', pady=(2, 0))
    qb_all = ttk.Button(quick, text='全选(8ch)', width=10, state='disabled',
                        command=lambda: on_sel_quick(list(range(8))))
    qb_all.pack(side='left', padx=3)
    sel_btns.append((qb_all, ()))            # 空 chs: 只随运行启停, 不参与高亮
    qb_clr = ttk.Button(quick, text='清空(静音)', width=11, state='disabled',
                        command=lambda: on_sel_quick([]))
    qb_clr.pack(side='left', padx=3)
    sel_btns.append((qb_clr, ()))
    # 父容器必须是 quick(自身一行): 挂 swf 会被当左条带, 文字变长会横向推挤按钮行
    ttk.Label(quick, textvariable=sel_lbl, foreground='blue', anchor='w').pack(
        side='left', fill='x', expand=True, padx=8)

    row0 = ttk.Frame(swf)
    row0.pack(fill='x', pady=2)
    for label, v in [('L/R', '0,1'), ('C/LFE', '2,3'),
                     ('SL/SR', '4,5'), ('SBL/SBR', '6,7')]:
        b = ttk.Button(row0, text=label, width=7, state='disabled',
                       command=lambda vv=v: on_sel_btn([int(x) for x in vv.split(',')]))
        b.pack(side='left', padx=3)
        sel_btns.append((b, tuple(int(x) for x in v.split(','))))
    row1 = ttk.Frame(swf)
    row1.pack(fill='x', pady=2)
    for c, nm in enumerate(['L', 'R', 'C', 'LFE', 'SL', 'SR', 'SBL', 'SBR']):
        b = ttk.Button(row1, text=nm, width=4, state='disabled',
                       command=lambda cc=c: on_sel_btn([cc]))
        b.pack(side='left', padx=3)
        sel_btns.append((b, (c,)))

    stf = ttk.LabelFrame(root, text=' 统计')
    stf.pack(fill='x', padx=10, pady=4)
    stat_vars = {k: tk.StringVar(value='-') for k in
                 ('rx', 'fec', 'late', 'loss', 'pct', 'buf', 't', 'un', 'lim', 'peak',
                  'e2e', 'net')}
    pct_lbl = None                       # 到达率标签(按阈值变色)
    val_lbls = {}                        # 值标签引用: 丢失/欠载/限幅异常着色用
    for row, items in enumerate(
            [[('rx', '接收包'), ('fec', 'FEC包'), ('late', '迟到'), ('loss', '丢失')],
             [('pct', '到达率'), ('buf', '缓冲(ms)'), ('t', '时长(s)'), ('un', '欠载')],
             [('lim', '限幅'), ('peak', '峰值dBFS')],
             [('e2e', '端到端(ms)'), ('net', '网络rtt(ms)')]]):
        for col, (key, label) in enumerate(items):
            ttk.Label(stf, text=label + ':').grid(row=row, column=col * 2, sticky='e',
                                                  padx=(8, 0), pady=4)
            l = ttk.Label(stf, textvariable=stat_vars[key], width=10, relief='flat',
                          background='#eef1f4', anchor='center',
                          font=('Consolas', 10, 'bold'))
            l.grid(row=row, column=col * 2 + 1, sticky='w')
            val_lbls[key] = l
            if key == 'pct':
                pct_lbl = l

    # 底部提示栏: 承接原参数区两段灰字说明, 常驻可见
    tk.Label(root, anchor='w', justify='left', foreground='gray',
             text='裸流=无 RTP 头 · 网卡IP: 多网卡机器必填 · 延迟档(预攒+在途ms): '
                  '低=20+40 / 正常=30+60 / 稳=60+150, 断续就升档 · 缓冲 低档约 30~70ms\n'
                  '按需单播(默认开)=发发现包收专属子集流·不组播(网卡IP必填, 声道选择即请求槽位) · '
                  '快捷键: 空格=开始/停止  1~8=切单声道  A=全选  C=清空'
             ).pack(side='bottom', fill='x', padx=10, pady=(0, 4))

    logf = ttk.LabelFrame(root, text=' 日志')
    logf.pack(fill='both', expand=True, padx=10, pady=4)
    txt = scrolledtext.ScrolledText(logf, height=8, state='disabled', font=('Consolas', 9))
    txt.pack(fill='both', expand=True)

    # 文件日志: 与 GUI 日志同内容落盘到脚本目录, 事后可回溯(丢包/缓冲/事件)
    # 注意: os 已在模块顶部导入, 这里不能再写 import os —— 那会把它变成 gui_main
    # 的局部名, 使前面 CFG_PATH 处报 UnboundLocalError(v5.3.0 首次运行踩到)
    # 固定文件名追加写(行首自带时间戳可回溯): 旧方案按秒取名每次运行一个新文件,
    # 无限累积; >8MB 时单级轮转成 .1(覆盖旧 .1), 磁盘占用封顶 ~16MB
    log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rxplay.log')
    try:
        if os.path.getsize(log_path) > 8 * 1024 * 1024:
            os.replace(log_path, log_path + '.1')   # 原子改名(旧 .1 直接被覆盖)
    except OSError:
        pass                                        # 文件不存在/改名失败: 不挡启动
    log_file = open(log_path, 'a', encoding='utf-8')

    # 闪退诊断(pythonw 无 stderr): Tk 回调异常 + 原生崩溃都落盘
    def _tk_exc(exc, val, tb):
        try:
            log_file.write(time.strftime('[%Y-%m-%d %H:%M:%S] ') +
                           'Tk 回调异常: %s: %s\n%s\n' %
                           (exc.__name__, val, ''.join(traceback.format_tb(tb))))
            log_file.flush()
        except Exception:
            pass
    root.report_callback_exception = _tk_exc
    try:
        faulthandler.enable(file=log_file)     # 原生崩溃(ctypes/winmm AV)也落盘
    except Exception:
        pass

    def do_log(msg):
        txt.configure(state='normal')
        txt.insert('end', time.strftime('[%H:%M:%S] ') + msg + '\n')
        txt.see('end')
        txt.configure(state='disabled')
        log_file.write(time.strftime('[%Y-%m-%d %H:%M:%S] ') + msg + '\n')
        log_file.flush()

    def _th_exhook(args):
        # 线程异常兜底: 写入文件日志(pythonw 无控制台, stderr 不可见);
        # 故意不碰 Tkinter 控件——跨线程 Tk 调用本身即是崩溃源
        try:
            log_file.write(time.strftime('[%Y-%m-%d %H:%M:%S] ') +
                           '线程异常[%s]: %s: %s\n' %
                           (getattr(args.exc_thread, 'name', '?'),
                            args.exc_type.__name__, args.exc_value))
            log_file.flush()
        except Exception:
            pass
    threading.excepthook = _th_exhook

    def worker_loop(cfg, ev):
        try:
            run_receiver(cfg, ev,
                         log=lambda m: ui_q.put(('log', m)),
                         stats=lambda s: ui_q.put(('stats', s)))
        except Exception as e:
            ui_q.put(('log', '错误: %s' % e))
        finally:
            ui_q.put(('log', '__DONE__'))

    def on_start():
        TX_OFF[0] = False                   # 重新开始: 取消上次挂起的停发通知
        try:
            ch_label = ch_var.get()
            chs = [int(x) for x in dict(CH_CHOICES)[ch_label].split(',')]
            port = int(port_var.get())
            rate = int(rate_var.get())
        except ValueError:
            do_log('参数错误: 端口/采样率须为数字')
            return
        if od_var.get() and not iface_ip():
            # 兜底再探一次(刚插网线/开热点后常见), 探测不到才让用户手填
            _v = iface_choices()
            if _v:
                iface_var.set(_v[0])
                do_log('未填网卡IP, 已自动选择: %s' % _v[0])
        if od_var.get() and not iface_ip():
            do_log('按需单播模式: 未探测到可用网卡, 请手动填写本机 IP(发现包需自报 IP)')
            return
        if od_var.get():
            try:
                zid = int(zone_var.get())
                assert 0 <= zid <= 255
            except (ValueError, AssertionError):
                do_log('按需单播模式: 区域ID 须为 0~255')
                return
        cfg = dict(port=port, group=group_var.get().strip(), rate=rate, chs=chs,
                   fmt=fmt_var.get(), raw=raw_var.get(), iface=iface_ip(),
                   profile={'极限': 'ultra', '低': 'low', '正常': 'normal', '稳': 'stable'}[lat_var.get()],
                   od=od_var.get(), zone=zone_var.get())
        save_cfg()                          # 记下本次参数, 下次启动直接恢复
        for w in (start_btn, rate_cb, ch_cb, iface_cb, lat_cb, od_ck, zone_entry):
            w.configure(state='disabled')
        stop_btn.configure(state='normal')
        state_lbl.configure(text='● 运行中', foreground='#2e7d32')
        set_sel(chs)                        # 同步初始监听 = 参数区选择
        sel_lbl.set('当前: ' + sel_desc())
        refresh_sel_buttons()
        for b in sel_btns:
            b[0].configure(state='normal')
        stop_holder[0] = threading.Event()   # 全新事件: 旧线程的旧 event 仍为 set, 各自退出
        for k in stat_vars:
            stat_vars[k].set('-')
        if pct_lbl:
            pct_lbl.configure(foreground='black')
        worker[0] = threading.Thread(target=worker_loop, args=(cfg, stop_holder[0]), daemon=True)
        worker[0].start()
        running[0] = True

    def on_stop():
        """停止: 先让接收线程退出(含 winmm 设备关闭), 停发通知延后到 poll 的 __DONE__。
        原因: 停止瞬间 waveOutReset/Close 是原生调用, 任何并发发包(内存分配)
        都会撞上堆损坏 -> Windows 原生 AV 闪退。故此处只置标志, 不碰 socket。"""
        try:
            was_running = running[0]
            running[0] = False
            if was_running and od_var.get():
                try:
                    zid = int(zone_var.get())
                except (TypeError, ValueError):
                    zid = 0
                TX_OFF[1], TX_OFF[2] = zid, iface_ip()
                TX_OFF[0] = True            # 交由 __DONE__ 在设备关闭后再发
            stop_holder[0].set()
            if was_running and od_var.get():
                do_log('停止中... 设备关闭后发送 tx_enable=0')
            stop_btn.configure(state='disabled')
            for b in sel_btns:
                b[0].configure(state='disabled')
            refresh_sel_buttons(clear=True)
            state_lbl.configure(text='● 停止中...', foreground='#ef6c00')
        except Exception:
            try:
                log_file.write(time.strftime('[%Y-%m-%d %H:%M:%S] ') +
                               '停止异常:\n' + traceback.format_exc())
                log_file.flush()
            except Exception:
                pass
            try:
                stop_holder[0].set()        # 无论如何都要让接收线程退出
            except Exception:
                pass

    start_btn.configure(command=on_start)
    stop_btn.configure(command=on_stop)

    # ---- 快捷键(焦点在输入框/下拉框时不抢, 免得打 IP 触发) ----
    def _typing():
        try:
            return isinstance(root.focus_get(), (tk.Entry, ttk.Entry, ttk.Combobox))
        except Exception:
            return False

    def _key_toggle(_e=None):
        if _typing():
            return
        if str(start_btn.cget('state')) != 'disabled':
            on_start()
        else:
            on_stop()

    def _key_ch(n):
        def h(_e=None):
            if not _typing():
                on_sel_btn([n])
        return h

    def _key_all(_e=None):
        if not _typing():
            on_sel_quick(list(range(8)))

    def _key_clear(_e=None):
        if not _typing():
            on_sel_quick([])

    root.bind('<space>', _key_toggle)
    for _n in range(8):
        root.bind(str(_n), _key_ch(_n))
    root.bind('<KeyPress-a>', _key_all)
    root.bind('<KeyPress-A>', _key_all)
    root.bind('<KeyPress-c>', _key_clear)
    root.bind('<KeyPress-C>', _key_clear)

    def poll():
        try:
            while True:
                kind, payload = ui_q.get_nowait()
                if kind == 'log':
                    if payload == '__DONE__':
                        # 此刻 worker 与其播放线程均已退出, winmm 设备已关闭 ——
                        # 停发通知(发包=内存分配)在这里做才不会与原生收尾并发 AV
                        if TX_OFF[0]:
                            tx_zid, tx_ifc = TX_OFF[1], TX_OFF[2]
                            TX_OFF[0] = False

                            def _tx_off():
                                time.sleep(0.25)          # 让 winmm 收尾彻底结束
                                if running[0]:            # 期间已重新开始: 不能停新流
                                    return
                                n = send_tx_beacon(tx_ifc, tx_zid, 0, n=3)
                                ui_q.put(('log', ('已通知板端停发流(tx_enable=0, %d 包)' % n)
                                          if n else '停发通知未发出, 板端将在 45s 老化后自行停止'))
                            threading.Thread(target=_tx_off, daemon=True).start()
                        start_btn.configure(state='normal')
                        rate_cb.configure(state='readonly')
                        ch_cb.configure(state='readonly')
                        lat_cb.configure(state='readonly')
                        iface_cb.configure(state='normal')   # on_start 禁用列表含它, 停止后须恢复
                        od_ck.configure(state='normal')
                        zone_entry.configure(state='normal')
                        for b in sel_btns:
                            b[0].configure(state='disabled')
                        refresh_sel_buttons(clear=True)
                        sel_lbl.set('未运行')
                        state_lbl.configure(text='● 未运行', foreground='gray')
                    else:
                        do_log(payload)
                elif kind == 'stats':
                    rawn = bool(payload.get('raw'))  # 裸流无 seq: 丢失/到达率无从计算,
                    stat_vars['rx'].set(str(payload['rx']))  # 显示"—", 不给假 0/100%
                    stat_vars['fec'].set(str(payload['fec']))
                    stat_vars['late'].set(str(payload['late']))
                    stat_vars['loss'].set('—' if rawn else str(payload['loss']))
                    stat_vars['pct'].set('—' if rawn else '%.1f%%' % payload['pct'])
                    stat_vars['buf'].set(str(payload.get('buf', '-')))
                    stat_vars['t'].set('%.0f' % payload['t'])
                    stat_vars['un'].set(str(payload.get('un', 0)))
                    stat_vars['lim'].set(str(payload.get('lim', 0)))
                    pk = payload.get('peak')
                    stat_vars['peak'].set('%.1f' % pk if pk is not None else '—')
                    # NTP 网络诊断(v5.4.0): e2e=发端采样→本机收到(NTP 换算,
                    # 含发端管线+网络); rtt=NTP 往返(纯网络); 未对时前显示"—"
                    ee = payload.get('e2e')
                    stat_vars['e2e'].set('%.0f' % ee if ee is not None else '—')
                    rr = payload.get('rtt')
                    stat_vars['net'].set('%.1f' % rr if rr is not None else '—')
                    val_lbls['loss'].configure(
                        foreground='#c62828' if (not rawn and payload['loss'])
                        else 'black')
                    val_lbls['un'].configure(
                        foreground='#ef6c00' if payload.get('un', 0) else 'black')
                    val_lbls['lim'].configure(
                        foreground='#ef6c00' if payload.get('lim', 0) else 'black')
                    val_lbls['peak'].configure(
                        foreground='#c62828' if (pk is not None and pk > -0.5)
                        else 'black')
                    if pct_lbl:                 # 到达率分级: 绿≥99.5 橙≥95 红<95
                        p = payload['pct']      # (裸流无 seq 不参与分级, 黑色"—")
                        pct_lbl.configure(foreground='black' if rawn else
                                          ('#2e7d32' if p >= 99.5 else
                                           ('#ef6c00' if p >= 95.0 else '#c62838')))
        except queue_mod.Empty:
            pass
        root.after(300, poll)

    do_log('提示: 格式默认 s32(现役定案档); 先选对采样率再开始; 首次运行防火墙弹窗选允许。'
           '声道按钮可多选(再点取消): ≥3 个=声像定位混音; 运行中随时可切换监听。')
    do_log('文件日志: %s' % log_path)

    # ---- 窗口尺寸: 优先恢复上次退出时的大小, 否则按控件实际需求自适应 ----
    # 不再按 DPI 乘系数: Tk 的字号本身已随缩放变大, 控件需求尺寸已经包含了缩放;
    # 再乘一次会让 150% 缩放下的窗口异常大。上下限都夹到屏幕内。
    root.update_idletasks()
    req_w, req_h = root.winfo_reqwidth(), root.winfo_reqheight()
    scr_w, scr_h = root.winfo_screenwidth(), root.winfo_screenheight()
    max_w, max_h = max(640, scr_w - 80), max(480, scr_h - 120)
    win_w = win_h = None
    if saved_geom[0]:
        m = re.match(r'^(\d+)x(\d+)', saved_geom[0])
        if m:
            win_w = min(max(int(m.group(1)), req_w), max_w)
            win_h = min(max(int(m.group(2)), req_h), max_h)
    if win_w is None:
        win_w = min(max(req_w + 16, 800), max_w)     # 刚好容纳控件 + 一点余量
        win_h = min(max(req_h + 100, 560), max_h)    # 日志区留 ~100px 可读高度
    root.geometry('%dx%d' % (win_w, win_h))
    root.minsize(min(req_w, max_w), min(req_h, max_h))

    root.after(300, poll)

    def on_close():
        """关窗: 与停止键同语义——od 模式先发 tx_enable=0 通知板端停发, 再退出。
        destroy 后 poll 随 mainloop 停转, __DONE__ 的延后发包路径不可达 ——
        此处同步等 worker(winmm 设备已关)退出后自己发, 时序与 poll/_tx_off 一致,
        否则板端会继续向本机空流 45s。"""
        try:
            on_stop()                      # 置 TX_OFF 标志 + 停接收线程(与停止键同路径)
        except Exception:
            pass
        if TX_OFF[0]:
            tx_zid, tx_ifc = TX_OFF[1], TX_OFF[2]
            TX_OFF[0] = False              # 摘牌: 防 destroy 前 poll 再发一次(重复无益)
            try:
                if worker[0] is not None:
                    worker[0].join(timeout=2.0)   # 等 run_receiver 收尾(含设备关闭)
            except Exception:
                pass
            if not running[0]:             # 等待期间未重新开始(理论上关窗不会)
                time.sleep(0.25)           # 让 winmm 收尾彻底结束(同 poll/_tx_off)
                n = send_tx_beacon(tx_ifc, tx_zid, 0, n=3)   # 内部吞异常, 不许崩
                try:
                    do_log('已通知板端停发流(tx_enable=0, %d 包)' % n)
                except Exception:
                    pass
        save_cfg()
        try:
            faulthandler.disable()         # 先摘原生崩溃落盘钩子再关文件: 否则此后
        except Exception:                  # 崩溃时 faulthandler 往已关闭文件写会二次崩
            pass
        log_file.close()
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', on_close)
    root.mainloop()

if __name__ == '__main__':
    if len(sys.argv) == 1:
        gui_main()          # 无参数 = GUI
    else:
        cli_main()
