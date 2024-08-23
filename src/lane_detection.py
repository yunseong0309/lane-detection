#!/usr/bin/env python3
#16,200,60
#20,200,50

import cv2
import numpy as np
import rospy
from std_msgs.msg import String
import matplotlib.pyplot as plt

#HSV변환
def detect_lane_HSV(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 흰색 범위 설정
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 25, 255])

    # 흰색 마스크 생성
    white_mask = cv2.inRange(hsv, lower_white, upper_white)

    # 흰색 검출
    white_result = cv2.bitwise_and(image, image, mask=white_mask)
    blurred_white_mask=cv2.GaussianBlur(white_mask, (5, 5), 0)

    return blurred_white_mask
#관심영역
def region_of_interest(edge_image):
    height, width = edge_image.shape[:2]

    # 관심 영역 설정 (사다리꼴)
    p1 = [0, 470]  # 좌하
    p2 = [90, 200]  # 좌상
    p3 = [540, 200]  # 우상
    p4 = [635, 470]  # 우하
    vertices = np.array([[p1, p2, p3, p4]], dtype=np.int32)

    # 비어 있는 영상 만들기
    mask = np.zeros_like(edge_image)

    # 다각형 내부를 색상으로 채우기
    cv2.fillConvexPoly(mask, vertices, 255)

    # 관심 영역 적용
    masked_image = cv2.bitwise_and(edge_image, mask)
    return masked_image
#원근 변환
def perspective_transform(image):
    height, width = image.shape[:2]

    p1 = [0, 470]  # 좌하
    p2 = [90, 200]  # 좌상
    p3 = [540, 200]  # 우상
    p4 = [635, 470]  # 우하

    src_points = np.float32([p2, p3, p4, p1])
    dst_points = np.float32([
        [0, 0],
        [width, 0],
        [width, height],
        [0, height]
    ])

    # 변환 행렬 계산
    matrix = cv2.getPerspectiveTransform(src_points, dst_points)
    matrixv = cv2.getPerspectiveTransform(dst_points, src_points)

    # 원근 변환 적용
    transformed_image = cv2.warpPerspective(image, matrix, (width, height), flags=cv2.INTER_LINEAR)
    return transformed_image, matrixv

#슬라이딩 윈도우
def sliding_window(image):
    output = np.dstack((image, image, image)) * 255
    histogram = np.sum(image[image.shape[0] // 2:, :], axis=0)

    midpoint = np.int(histogram.shape[0] / 2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    num_windows = 20
    window_height = np.int(image.shape[0] / num_windows)
    nonzero = image.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])

    current_leftX = leftx_base
    current_rightX = rightx_base

    min_num_pixel = 200
    window_margin = 50

    win_left_lane = []
    win_right_lane = []

    for window in range(num_windows):
        win_y_low = image.shape[0] - (window + 1) * window_height
        win_y_high = image.shape[0] - window * window_height
        win_leftx_min = current_leftX - window_margin
        win_leftx_max = current_leftX + window_margin
        win_rightx_min = current_rightX - window_margin
        win_rightx_max = current_rightX + window_margin

        cv2.rectangle(output, (win_leftx_min, win_y_low), (win_leftx_max, win_y_high), (0, 255, 0), 2)
        cv2.rectangle(output, (win_rightx_min, win_y_low), (win_rightx_max, win_y_high), (0, 255, 0), 2)

        left_window_inds = ((nonzeroy >= win_y_low) & (nonzeroy <= win_y_high) & 
                            (nonzerox >= win_leftx_min) & (nonzerox <= win_leftx_max)).nonzero()[0]
        right_window_inds = ((nonzeroy >= win_y_low) & (nonzeroy <= win_y_high) & 
                             (nonzerox >= win_rightx_min) & (nonzerox <= win_rightx_max)).nonzero()[0]

        if len(left_window_inds) > min_num_pixel:
            current_leftX = np.int(np.mean(nonzerox[left_window_inds]))
            win_left_lane.append(left_window_inds)

        if len(right_window_inds) > min_num_pixel:
            current_rightX = np.int(np.mean(nonzerox[right_window_inds]))
            win_right_lane.append(right_window_inds)

        # 만약 왼쪽과 오른쪽이 너무 가까우면 하나의 차선으로 인식
        if abs(current_leftX - current_rightX) < window_margin:
            if len(left_window_inds) > len(right_window_inds):
                current_rightX = current_leftX  # 오른쪽을 왼쪽에 맞춤
            else:
                current_leftX = current_rightX  # 왼쪽을 오른쪽에 맞춤

    win_left_lane = np.concatenate(win_left_lane) if len(win_left_lane) > 0 else []
    win_right_lane = np.concatenate(win_right_lane) if len(win_right_lane) > 0 else []

    leftx, lefty = nonzerox[win_left_lane], nonzeroy[win_left_lane] if len(win_left_lane) > 0 else ([], [])
    rightx, righty = nonzerox[win_right_lane], nonzeroy[win_right_lane] if len(win_right_lane) > 0 else ([], [])

    # if len(leftx) > 0 and len(rightx) > 0 and abs(current_leftX - current_rightX) < window_margin:
    #     if len(leftx) > len(rightx):  # 왼쪽이 더 많으면 왼쪽 차선으로 인식
    #         leftx = np.concatenate((leftx, rightx))
    #         lefty = np.concatenate((lefty, righty))
    #         rightx, righty = [], []
    #     else:  # 오른쪽이 더 많으면 오른쪽 차선으로 인식
    #         rightx = np.concatenate((rightx, leftx))
    #         righty = np.concatenate((righty, lefty))
    #         leftx, lefty = [], []

    # 하나의 차선으로 통합
    if len(leftx) > 0 and len(rightx) > 0 and abs(current_leftX - current_rightX) < window_margin:
        leftx = np.concatenate((leftx, rightx))
        lefty = np.concatenate((lefty, righty))
        rightx, righty = [], []

    if len(leftx) == 0 and len(rightx) == 0:
        return None, None, None, None, None, None
    
    left_fit = np.polyfit(lefty, leftx, 2) if len(leftx) > 0 else None
    right_fit = np.polyfit(righty, rightx, 2) if len(rightx) > 0 else None

    if len(leftx) > 0:
        output[lefty, leftx] = [255, 0, 0]
    if len(rightx) > 0:
        output[righty, rightx] = [0, 0, 255]
    
    cv2.imshow('Sliding Window', output)

    return left_fit, right_fit, leftx, lefty, rightx, righty

#도로영역 시각화
def draw_lane(image, wrap_img, matrixv, left_fit, right_fit):
    height, width = wrap_img.shape[:2]
    if left_fit is None or right_fit is None:
        return image
    yMax = wrap_img.shape[0]
    ploty = np.linspace(0, yMax - 1, yMax)
    color_warp = np.zeros((yMax, wrap_img.shape[1], 3), dtype=np.uint8)

    left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]
    right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]

    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))]) 
    pts = np.hstack((pts_left, pts_right))

    color_warp = cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
    newwarp = cv2.warpPerspective(color_warp, matrixv, (width, height))

    result = cv2.addWeighted(image, 1, newwarp, 0.3, 0)
    return result

def steering_control(cap):
    rospy.init_node('lane_following_control')
    pub = rospy.Publisher('/control', String, queue_size=10)
    
    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        ret, image = cap.read()
        if not ret:
            rospy.logwarn("프레임을 읽을 수 없습니다.")
            #break

        image_hsv = detect_lane_HSV(image)
        #gray_image_hsv = cv2.cvtColor(image_hsv, cv2.COLOR_BGR2GRAY)
        #canny_image = cv2.Canny(image_hsv, 30, 300)
        roi_image = region_of_interest(image_hsv)
        bev_image, matrixv = perspective_transform(roi_image)

        # 차선 인식 및 좌표 계산
        left_fit, right_fit, leftx, lefty, rightx, righty = sliding_window(bev_image)
        print("leftx")
        print(leftx)
        print("rightx____")
        print(rightx)
        if leftx is None and rightx is None:
            rospy.logerr("No lanes detected! Stopping the vehicle.")
            pub.publish("Stop")
            break

        if len(leftx) > 0 and len(rightx) == 0:
            rospy.logwarn("Left lane detected only, turning right")
            pub.publish("Right")
        elif len(rightx) > 0 and len(leftx) == 0:
            rospy.logwarn("Right lane detected only, turning left")
            pub.publish("Left")
        else:
            leftx_median = np.median(leftx)
            rightx_median = np.median(rightx)
            lane_center_x = (leftx_median + rightx_median) / 2
            
            vehicle_x = bev_image.shape[1] / 2  # 차량이 이미지 중앙에 있다고 가정
            offset = lane_center_x - vehicle_x  # 오차 계산

            if offset < -50:
                rospy.loginfo("Turn left")
                pub.publish("Left")
            elif offset > 50:
                rospy.loginfo("Turn right")
                pub.publish("Right")
            else:
                rospy.loginfo("Go straight")
                pub.publish("Go")

        # 차선 시각화
        lane_img = draw_lane(image, bev_image, matrixv, left_fit, right_fit)

        # 시각화 창 업데이트
        cv2.imshow('Lane Detection', lane_img)
        #cv2.imshow('Original', image)
        # plt.imshow(lane_img)
        # plt.show()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        rate.sleep()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        cap = cv2.VideoCapture(2)
        if not cap.isOpened():
            print("웹캠을 열 수 없습니다.")
            exit()

        steering_control(cap)
    except rospy.ROSInterruptException:
        pass
    finally:
        cap.release()
