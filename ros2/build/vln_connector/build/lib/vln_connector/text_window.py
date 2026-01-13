import cv2

class TextVisualizer:
    def __init__(self, width=800, height=600):
        self.width = width
        self.height = height
        self.lines = []

    def show_text(self, text):
        self.lines.append(text)
        # 保留最近 20 行
        self.lines = self.lines[-20:]
        
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        y0, dy = 30, 25
        for i, line in enumerate(self.lines):
            y = y0 + i*dy
            cv2.putText(img, line[:100], (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1)
        cv2.imshow("LLM Output", img)
        cv2.waitKey(1)
