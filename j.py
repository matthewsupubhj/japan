#!/usr/bin/env python3
# 指定此腳本使用 Python 3 解釋器執行

import rospy  # ROS 的 Python 客戶端庫，用於與 ROS 節點交互
from sensor_msgs.msg import Image  # ROS 消息類型，處理相機的圖像數據
from cv_bridge import CvBridge  # 用於將 ROS 的 Image 消息與 OpenCV 圖像進行轉換
import cv2  # OpenCV 庫，用於圖像處理
import numpy as np  # 用於數據處理（例如計算矩心）

# 回調函數，處理接收到的圖像消息
def callback_image(msg):
    """
    回調函數，當接收到指定主題的圖像消息時執行。
    將 ROS 的 Image 消息轉換為 OpenCV 格式的 BGR 圖像。
    """
    global frame  # 使用全局變量存儲最新的圖像數據
    frame = CvBridge().imgmsg_to_cv2(msg, "bgr8")  # 將 ROS 圖像消息轉換為 OpenCV 圖像

if __name__ == "__main__":
    # 初始化 ROS 節點
    rospy.init_node("ros_tutorial")  # 初始化節點名稱為 "ros_tutorial"
    rospy.loginfo("ros_tutorial node start!")  # 打印節點啟動信息
    
    frame = None  # 初始化存儲圖像的全局變量
    topic_name = "/camera/rgb/image_raw"  # 訂閱的圖像主題名稱
    # 訂閱指定主題，當接收到消息時，執行回調函數 callback_image
    rospy.Subscriber(topic_name, Image, callback_image)
    
    # 等待主題的第一條消息，確保程序獲得數據後才進入主循環
    rospy.wait_for_message(topic_name, Image)
    
    # 主循環，處理並顯示圖像
    while not rospy.is_shutdown():  # 當節點未被關閉時執行
        rospy.Rate(20).sleep()  # 設置循環頻率為 20 Hz
        
        # 將 BGR 格式的圖像轉換為 HSV 格式，用於顏色檢測
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 定義黃色的 HSV 範圍（22~32 的色調範圍）
        lower = (22, 100, 100)  # 黃色的下限 HSV 值
        upper = (32, 255, 255)  # 黃色的上限 HSV 值
        mask = cv2.inRange(hsv, lower, upper)  # 創建遮罩，篩選出黃色區域
        
        # 查找遮罩中的輪廓
        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            # 計算輪廓的面積
            area = cv2.contourArea(contour)
            if area > 500:  # 忽略小的輪廓，只處理面積大於 500 的輪廓
                # 繪製輪廓
                cv2.drawContours(frame, [contour], 0, (0, 0, 255), 2)  # 紅色的輪廓線
                
                # 繪製外接矩形
                x, y, w, h = cv2.boundingRect(contour)  # 計算外接矩形參數
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)  # 藍色矩形框
                
                # 計算輪廓的中心點
                cx, cy = 0, 0  # 初始化中心點
                m = cv2.moments(contour)  # 計算輪廓的幾何矩
                if m["m00"] != 0:  # 確保 m["m00"] 不為零，避免除零錯誤
                    cx = int(np.round(m["m10"] / m["m00"]))  # x 坐標
                    cy = int(np.round(m["m01"] / m["m00"]))  # y 坐標
                
                # 繪製中心點
                cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)  # 綠色的圓形中心點
        
        # 顯示處理後的圖像
        cv2.imshow("frame", frame)  # 在窗口中顯示圖像
        key_code = cv2.waitKey(1)  # 等待按鍵輸入，間隔 1 毫秒
        if key_code in [ord('q'), 27]:  # 如果按下 'q' 或 'Esc' 鍵，退出循環
            break
    
    # 關閉所有 OpenCV 窗口
    cv2.destroyAllWindows()
    rospy.loginfo("ros_tutorial node end!")  # 打印節點結束信息
