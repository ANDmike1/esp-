# OpenMV IDE — 人像大致方位（相对画面中线：左 / 右）
# 肤色 LAB 检测，无需 Haar/SD。UART 每行一个字母。
# P4=TX P5=RX，波特率见 UART_BAUD。

import sensor
import image
import time
import pyb

# ---------- 可调 ----------
FRAME_SIZE = sensor.QVGA
USE_UART = True
UART_BAUD = 115200

# 中线附近宽度（像素）：在此宽度内认为「正中」，发 C，减轻抖动；设为 0 则只发 L/R（以宽度一半为界）
CENTER_DEAD_PX = 16

SKIN_THRESHOLDS = [
    (38, 74, -21, 38, -49, 39),
    (0, 70, -15, 25, -25, 25),
]
SKIN_PIXELS_THRESHOLD = 280
SKIN_BLOB_MERGE_MARGIN = 12
MIN_BLOB_WH = (36, 36)


def setup_sensor():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(FRAME_SIZE)
    sensor.skip_frames(time=400)
    sensor.set_auto_gain(True)
    sensor.set_auto_whitebal(True)


def largest_skin_rect(img):
    blobs = img.find_blobs(
        SKIN_THRESHOLDS,
        pixels_threshold=SKIN_PIXELS_THRESHOLD,
        area_threshold=SKIN_PIXELS_THRESHOLD,
        merge=True,
        margin=SKIN_BLOB_MERGE_MARGIN,
    )
    if not blobs:
        return None
    mw, mh = MIN_BLOB_WH
    cand = [b for b in blobs if b.w() >= mw and b.h() >= mh] or blobs
    best = max(cand, key=lambda b: b.pixels())
    return best.rect()


def side_letter(cx, w_frame, dead):
    """相对竖直中线：'L' 'R' 或 'C'（死区内）。"""
    mid = w_frame // 2
    if dead <= 0:
        return "L" if cx < mid else "R"
    if cx < mid - dead:
        return "L"
    if cx > mid + dead:
        return "R"
    return "C"


def main():
    setup_sensor()
    w = sensor.width()
    h = sensor.height()
    mid = w // 2

    uart = None
    if USE_UART:
        uart = pyb.UART(3, UART_BAUD, timeout_char=50)

    while True:
        img = sensor.snapshot()
        rect = largest_skin_rect(img)
        if rect is None:
            ch = "N"
        else:
            x, y, rw, rh = rect
            cx = x + rw // 2
            cy = y + rh // 2
            img.draw_rectangle(rect, color=(0, 255, 0))
            img.draw_cross(cx, cy, color=(0, 255, 0))
            ch = side_letter(cx, w, CENTER_DEAD_PX)

        # 画一条竖直中线参考
        img.draw_line(mid, 0, mid, h - 1, color=(200, 200, 200))
        img.draw_string(2, 2, ch, color=(255, 255, 255), scale=2)

        if USE_UART and uart is not None:
            uart.write("%s\r\n" % ch)


if __name__ == "__main__":
    main()
