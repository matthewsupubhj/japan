
#!/usr/bin/env python3
# 指定此腳本使用 Python 3 解釋器執行

import rospy  # ROS 的 Python 客戶端庫，用於與 ROS 节点交互
from sensor_msgs.msg import Image  # ROS 消息類型，用於處理相機圖像數據
from cv_bridge import CvBridge  # 用於在 ROS 的 Image 消息和 OpenCV 圖像之間進行轉換
import cv2  # OpenCV 庫，用於圖像處理

# 回調函數，用於接收並處理圖像數據
def callback_image(msg):
    """
    回調函數，當接收到 /camera/rgb/image_raw 主題的消息時執行。
    將 ROS 的 Image 消息轉換為 OpenCV 格式的圖像。
    """
    global frame  # 使用全局變量存儲最新的圖像數據
    frame = CvBridge().imgmsg_to_cv2(msg, "bgr8")  # 將 ROS 消息轉換為 OpenCV 圖像

if __name__ == "__main__":
    # 初始化 ROS 節點
    rospy.init_node("ros_tutorial")  # 節點名稱為 "ros_tutorial"
    rospy.loginfo("ros_tutorial node start!")  # 打印節點啟動信息
    
    frame = None  # 初始化存儲圖像的變量，默認為空
    topic_name = "/camera/rgb/image_raw"  # 訂閱的圖像主題名稱
    # 注意：根據相機設備，可能需要更改主題名，例如 "/camera/color/image_raw"
    
    # 訂閱圖像主題，當接收到消息時，執行回調函數 callback_image
    rospy.Subscriber(topic_name, Image, callback_image)
    
    # 等待主題的第一條消息，以確保節點正常接收到數據
    rospy.wait_for_message(topic_name, Image)
    
    # 主循環，持續顯示接收到的圖像
    while not rospy.is_shutdown():  # 當節點未被關閉時執行
        rospy.Rate(20).sleep()  # 設置循環頻率為 20 Hz
        
        # 使用 OpenCV 顯示圖像窗口
        cv2.imshow("frame", frame)  # 顯示最新的圖像數據
        key_code = cv2.waitKey(1)  # 等待按鍵輸入，間隔 1 毫秒
        if key_code in [ord('q'), 27]:  # 如果按下 'q' 或 'Esc' 鍵，退出循環
            break
    
    # 關閉所有 OpenCV 窗口
    cv2.destroyAllWindows()
    rospy.loginfo("ros_tutorial node end!")  # 打印節點結束信息
