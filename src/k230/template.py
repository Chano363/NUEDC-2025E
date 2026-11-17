from libs.PipeLine import PipeLine, ScopedTiming
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
import os,sys,gc,time,random,utime,math
import ujson
from media.sensor import Sensor
from media.media import *
from time import *
import nncase_runtime as nn
import ulab.numpy as np
import image

class YOLOApp(AIBase):
    def __init__(self, kmodel_path, model_input_size, confidence_threshold=0.5, nms_threshold=0.45, rgb888p_size=[320,240], display_size=[640,480], classes=("ball",), debug_mode=0):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        self.kmodel_path = kmodel_path
        self.model_input_size = model_input_size
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]
        self.classes = classes
        self.num_classes = len(classes)
        self.debug_mode = debug_mode
        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)
        self._pad_top = 0
        self._pad_bottom = 0
        self._pad_left = 0
        self._pad_right = 0

    # 配置预处理操作：先pad再resize，构成letterbox
    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            self._pad_top, self._pad_bottom, self._pad_left, self._pad_right = self.get_padding_param()
            print("padding: {} {} {} {}".format(self._pad_top, self._pad_bottom, self._pad_left, self._pad_right))
            self.ai2d.pad([0, 0, 0, 0, self._pad_top, self._pad_bottom, self._pad_left, self._pad_right], 0, [104, 117, 123])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],[1,3,self.model_input_size[1],self.model_input_size[0]])

    # 自定义当前任务的后处理
    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            dets = []
            try:
                if results and len(results) > 0:
                    output = results[0]
                    output_shape = output.shape
                    if len(output_shape) == 3 and output_shape[1] > 4:
                        # YOLO输出 [1,4+num_classes,N]: 0-3=xywh(中心点), 4+=各类logits
                        num_cells = output_shape[2]

                        # 向量化求最佳类别（ulab C级运算）
                        scores = output[0, 4:, :]
                        max_scores_arr = np.max(scores, axis=0)
                        max_classes_arr = np.argmax(scores, axis=0)

                        # 模型空间中有效区域高度（去除padding）
                        valid_h = self.model_input_size[1] - self._pad_top - self._pad_bottom

                        for j in range(num_cells):
                            # 跳过padding区域的网格
                            if float(output[0, 1, j]) >= valid_h:
                                continue
                            conf = self._sigmoid(float(max_scores_arr[j]))
                            if conf >= self.confidence_threshold:
                                cx = float(output[0, 0, j])
                                cy = float(output[0, 1, j])
                                w = float(output[0, 2, j])
                                h = float(output[0, 3, j])
                                # 坐标从模型空间映射到相机空间
                                scale_x = self.rgb888p_size[0] / self.model_input_size[0]
                                scale_y = self.rgb888p_size[1] / valid_h
                                cx = cx * scale_x
                                cy = cy * scale_y
                                w = w * scale_x
                                h = h * scale_y
                                dets.append([cx, cy, w, h, conf, float(max_classes_arr[j])])

                    # NMS
                    dets = self._non_max_suppression(dets)
            except Exception as e:
                print("Postprocess error: {}".format(e))
            return dets

    @staticmethod
    def _sigmoid(x):
        if x < -88: return 0.0
        if x > 88:  return 1.0
        return 1.0 / (1.0 + math.exp(-x))

    def _non_max_suppression(self, dets):
        if len(dets) == 0: return []
        dets.sort(key=lambda x: x[4], reverse=True)
        keep = []
        while dets:
            keep.append(dets[0])
            curr = dets[0]
            dets = dets[1:]
            remaining = []
            for d in dets:
                iou = self._calculate_iou(curr[:4], d[:4])
                if iou < self.nms_threshold:
                    remaining.append(d)
            dets = remaining
        return keep

    @staticmethod
    def _calculate_iou(box1, box2):
        # box: [cx, cy, w, h]
        x1, y1 = box1[0] - box1[2] / 2, box1[1] - box1[3] / 2
        x2, y2 = box2[0] - box2[2] / 2, box2[1] - box2[3] / 2
        ix1, iy1 = max(x1, x2), max(y1, y2)
        ix2, iy2 = min(x1 + box1[2], x2 + box2[2]), min(y1 + box1[3], y2 + box2[3])
        inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
        union = box1[2] * box1[3] + box2[2] * box2[3] - inter
        return inter / union if union > 0 else 0

    # 绘制检测结果到画面上
    def draw_result(self, pl, dets):
        with ScopedTiming("display_draw", self.debug_mode > 0):
            if dets:
                pl.osd_img.clear()
                for det in dets:
                    # det: [cx, cy, w, h, conf, cls_id] — 已在相机空间(320×240)
                    cx, cy, w, h = map(lambda v: int(round(v, 0)), det[:4])
                    conf = det[4]
                    cls_id = int(det[5]) if len(det) > 5 else 0

                    if self.debug_mode:
                        print("[DEBUG] cam:({},{},{},{}) conf={:.2f} cls={}".format(cx, cy, w, h, conf, cls_id))

                    # 模型输出是中心点坐标(cx,cy)，转换为左上角(x,y)用于绘制
                    x_tl = cx - w // 2
                    y_tl = cy - h // 2

                    # 缩放到显示尺寸
                    x_disp = x_tl * self.display_size[0] // self.rgb888p_size[0]
                    y_disp = y_tl * self.display_size[1] // self.rgb888p_size[1]
                    w_disp = w * self.display_size[0] // self.rgb888p_size[0]
                    h_disp = h * self.display_size[1] // self.rgb888p_size[1]

                    # 绘制矩形框
                    pl.osd_img.draw_rectangle(x_disp, y_disp, w_disp, h_disp, color=(255, 0, 255, 0), thickness=2)
                    # 绘制标签
                    cls_name = self.classes[cls_id] if cls_id < len(self.classes) else str(cls_id)
                    label = "{} {:.0f}%".format(cls_name, conf * 100)
                    pl.osd_img.draw_string_advanced(x_disp, y_disp - 20 if y_disp >= 20 else y_disp, 40, label, color=(255, 0, 255, 0))
            else:
                pl.osd_img.clear()

    # 获取padding参数
    def get_padding_param(self):
        dst_w = self.model_input_size[0]
        dst_h = self.model_input_size[1]
        ratio_w = dst_w / self.rgb888p_size[0]
        ratio_h = dst_h / self.rgb888p_size[1]
        ratio = min(ratio_w, ratio_h)
        new_w = int(ratio * self.rgb888p_size[0])
        new_h = int(ratio * self.rgb888p_size[1])
        dw = (dst_w - new_w) / 2
        dh = (dst_h - new_h) / 2
        top = int(round(0))
        bottom = int(round(dh * 2 + 0.1))
        left = int(round(0))
        right = int(round(dw * 2 - 0.1))
        return top, bottom, left, right

if __name__ == "__main__":
    # 与 CV_lite混合.py 对齐：ST7701 @ 640x480, to_ide=True
    display_size = [640, 480]
    rgb888p_size = [640, 480]

    class_names = ("ball",)
    kmodel_path = "/sdcard/best.kmodel"
    model_input_size = [416, 416]
    confidence_threshold = 0.6 #只允许人工修改。
    nms_threshold = 0.45

    pl = PipeLine(rgb888p_size=rgb888p_size, display_size=display_size, display_mode='st7701')
    pl.create(Sensor(width=1280, height=960))
    yolo_det = YOLOApp(kmodel_path, model_input_size=model_input_size, confidence_threshold=confidence_threshold, nms_threshold=nms_threshold, rgb888p_size=rgb888p_size, display_size=display_size, classes=class_names, debug_mode=0)
    yolo_det.config_preprocess()

    MAX_LOST = 6
    INFER_INTERVAL = 3
    TRACK_DIST = 80          # 最大关联距离（像素）
    tracks = []              # [cx,cy,w,h,conf,cls, vx,vy, lost]

    infer_skip = 0
    fps_counter = 0
    fps = 0.0
    last_tick = utime.ticks_ms()

    try:
        while True:
            os.exitpoint()
            with ScopedTiming("total", 1):
                img = pl.get_frame()

                infer_skip += 1
                do_infer = (infer_skip >= INFER_INTERVAL)
                if do_infer:
                    infer_skip = 0
                    res = yolo_det.run(img)
                    detected = [list(d[:6]) for d in res] if res else []

                    matched_t = set()
                    matched_d = set()
                    for ti in range(len(tracks)):
                        best_di = -1
                        best_d2 = TRACK_DIST * TRACK_DIST
                        for di in range(len(detected)):
                            if di in matched_d:
                                continue
                            dx = tracks[ti][0] - detected[di][0]
                            dy = tracks[ti][1] - detected[di][1]
                            d2 = dx * dx + dy * dy
                            if d2 < best_d2:
                                best_d2 = d2
                                best_di = di
                        if best_di >= 0:
                            t = tracks[ti]; d = detected[best_di]
                            new_vx = (d[0] - t[0]) / INFER_INTERVAL
                            new_vy = (d[1] - t[1]) / INFER_INTERVAL
                            t[6] = t[6] * 0.4 + new_vx * 0.6
                            t[7] = t[7] * 0.4 + new_vy * 0.6
                            t[0],t[1] = d[0],d[1]
                            t[2],t[3] = d[2],d[3]
                            t[4],t[5] = d[4],d[5]
                            t[8] = 0
                            matched_t.add(ti)
                            matched_d.add(best_di)

                    for di in range(len(detected)):
                        if di not in matched_d:
                            d = detected[di]
                            tracks.append([d[0],d[1],d[2],d[3],d[4],d[5], 0.0,0.0, 0])

                    for ti in range(len(tracks)):
                        if ti not in matched_t:
                            t = tracks[ti]
                            t[0] += t[6]; t[1] += t[7]
                            t[6] *= 0.85; t[7] *= 0.85
                            t[8] += 1

                    tracks = [t for t in tracks if t[8] < MAX_LOST]

                else:
                    for t in tracks:
                        t[0] += t[6]; t[1] += t[7]
                        t[6] *= 0.85; t[7] *= 0.85
                        t[8] += 1
                    tracks = [t for t in tracks if t[8] < MAX_LOST]

                draw_dets = []
                for t in tracks:
                    cx = max(0, min(int(t[0]), rgb888p_size[0]))
                    cy = max(0, min(int(t[1]), rgb888p_size[1]))
                    draw_dets.append([cx, cy, int(t[2]), int(t[3]), t[4], int(t[5])])
                yolo_det.draw_result(pl, draw_dets)

                fps_counter += 1
                now = utime.ticks_ms()
                elapsed = utime.ticks_diff(now, last_tick)
                if elapsed >= 1000:
                    fps = fps_counter * 1000.0 / elapsed
                    fps_counter = 0
                    last_tick = now

                mode_str = "INFER" if do_infer else "PREDICT"
                pl.osd_img.draw_string_advanced(5, 5, 20, "FPS:{:.1f} {} trk:{}".format(fps, mode_str, len(tracks)), color=(255, 0, 255, 0))

                pl.show_image()
                gc.collect()
    except Exception as e:
        print(e)
    finally:
        yolo_det.deinit()
        pl.destroy()
