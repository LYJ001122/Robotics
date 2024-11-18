import pyrealsense2 as rs
import cv2
import numpy as np

class TemplateMatcher:
    def __init__(self, template_paths, score_threshold=0.5, size_threshold=10, scale_range=(0.025, 1, 0.025)):
        self.templates = self.load_and_resize_templates(template_paths)
        self.score_threshold = score_threshold
        self.size_threshold = size_threshold
        self.scale_range = scale_range

        # Realsense 카메라 설정
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    def load_and_resize_templates(self, template_paths):
        ori_templates = [cv2.imread(path, cv2.IMREAD_GRAYSCALE) for path in template_paths]
        template_2_height = ori_templates[1].shape[0] // 2  # 기준 템플릿 높이의 절반
        resized_templates = []

        for idx, template in enumerate(ori_templates):
            h, w = template.shape
            scale_factor = template_2_height / h
            new_width = int(w * scale_factor)
            resized_template = cv2.resize(template, (new_width, template_2_height), interpolation=cv2.INTER_LINEAR)
            resized_templates.append(resized_template)

        return resized_templates

    def start_camera(self):
        self.pipeline.start(self.config)

    def stop_camera(self):
        self.pipeline.stop()

    def multi_scale_template_matching(self, image):
        best_match = None
        scales = np.arange(*self.scale_range)

        for idx, template in enumerate(self.templates):
            for scale in scales:
                resized_template = cv2.resize(template, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)

                if resized_template.shape[0] <= image.shape[0] and resized_template.shape[1] <= image.shape[1]:
                    result = cv2.matchTemplate(image, resized_template, cv2.TM_CCOEFF_NORMED)
                    _, max_val, _, max_loc = cv2.minMaxLoc(result)

                    if max_val >= self.score_threshold and resized_template.shape[0] >= self.size_threshold and resized_template.shape[1] >= self.size_threshold:
                        if best_match is None or max_val > best_match["score"]:
                            best_match = {
                                "template_idx": idx,
                                "score": max_val,
                                "top_left": max_loc,
                                "template_size": resized_template.shape,
                                "scale": scale
                            }

        return best_match

    def get_center_point(self, match_result):
        if match_result:
            top_left = match_result["top_left"]
            template_size = match_result["template_size"]
            bottom_right = (top_left[0] + template_size[1], top_left[1] + template_size[0])
            center_x = (top_left[0] + bottom_right[0]) // 2
            center_y = (top_left[1] + bottom_right[1]) // 2
            return center_x, center_y
        return None

    def run(self):
        self.start_camera()
        try:
            while True:
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                color_image = np.asanyarray(color_frame.get_data())
                gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

                best_match = self.multi_scale_template_matching(gray_image)
                center_point = self.get_center_point(best_match)
                
                if center_point:

                if center_point:

                    
                # if center_point:
                #     print(f"Template {best_match['template_idx'] + 1} Center: {center_point}, Score: {best_match['score']:.2f}")

                #     top_left = best_match["top_left"]
                #     template_size = best_match["template_size"]
                #     bottom_right = (top_left[0] + template_size[1], top_left[1] + template_size[0])
                #     cv2.rectangle(color_image, top_left, bottom_right, (0, 255, 0), 2)
                #     cv2.putText(color_image, f'Center: {center_point}, Score: {best_match["score"]:.2f}',
                #                 (top_left[0], top_left[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                # cv2.imshow('Template Matching', color_image)
                

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            self.stop_camera()
            cv2.destroyAllWindows()


template_paths = ['template_1.png', 'template_2.png', 'template_3.png', 'template_4.jpg']

matcher = TemplateMatcher(template_paths, score_threshold=0.5, size_threshold=10, scale_range=(0.025, 1, 0.025))

matcher.run()
