import cv2
import numpy as np
import matplotlib.pyplot as plt

# 전역 변수로 이전 프레임의 차선 위치를 저장
prev_left_base = None
prev_right_base = None

# Fixed Bird Eye's View Points
fixed_points = [(10, 450), (540, 430), (435, 240), (190, 240)]

# 픽셀 당 거리(m/pixel)
x_m_per_pixel = 0.0016472868217054263
y_m_per_pixel = 0.003703703703703704
real_shift_distance = 0.45  # 실제 이동해야 할 거리 (0.45m, 즉 450mm)
L_m = 55 * 10 ** -2  # 차량의 wheel base (0.55m)


def get_bird_eye_view(image, output_size, points):
    if len(points) != 4:
        return image

    height, width = output_size[1], output_size[0]
    src_points = np.float32([points[0], points[1], points[3], points[2]])
    dst_points = np.float32([[0, height], [width, height], [0, 0], [width, 0]])

    matrix = cv2.getPerspectiveTransform(src_points, dst_points)
    bird_eye_view = cv2.warpPerspective(image, matrix, output_size)

    return bird_eye_view


def mask_hls_areas(image, lower_hls, upper_hls, blur_ksize=(23, 23)):
    blurred_image = cv2.GaussianBlur(image, blur_ksize, 0)
    hls_image = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2HLS)
    mask = cv2.inRange(hls_image, np.array(lower_hls), np.array(upper_hls))
    return mask


def remove_circular_objects(mask, min_radius=25, max_radius=56):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        if len(contour) >= 5:
            (x, y), radius = cv2.minEnclosingCircle(contour)
            if min_radius < radius < max_radius:
                cv2.drawContours(mask, [contour], -1, 0, -1)
    return mask


def sliding_window_demo(image, right_nwindows=18, left_nwindows=6, right_margin=60, left_margin=150, minpix=50,
                        discontinuity_windows=2):
    global prev_left_base, prev_right_base

    lower_hls = [0, 200, 0]
    upper_hls = [180, 255, 255]

    masked_image = mask_hls_areas(image, lower_hls, upper_hls)
    masked_image = remove_circular_objects(masked_image)
    base_limit_height = masked_image.shape[0] * 4 // 5
    histogram = np.sum(masked_image[base_limit_height:, :], axis=0)

    midpoint = int(histogram.shape[0] // 2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    left_base_boundary = 1 * image.shape[1] // 3
    right_base_boundary = 1.7 * image.shape[1] // 3

    apply_left_window = left_base_boundary >= leftx_base > 0
    apply_right_window = rightx_base >= right_base_boundary

    if not apply_left_window and prev_left_base is not None:
        leftx_base = prev_left_base
    else:
        prev_left_base = leftx_base

    if not apply_right_window and prev_right_base is not None:
        rightx_base = prev_right_base
    else:
        prev_right_base = rightx_base

    left_window_height = int(masked_image.shape[0] // left_nwindows)
    right_window_height = int(masked_image.shape[0] // right_nwindows)

    leftx_current = leftx_base
    rightx_current = rightx_base

    out_img = np.dstack((masked_image, masked_image, masked_image))

    nonzero = masked_image.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])

    left_lane_points_x = []
    left_lane_points_y = []
    right_lane_points_x = []
    right_lane_points_y = []

    left_windows_lower_bound = image.shape[0] * 1 // 4  # 하단 3/4의 시작점 (360)
    left_windows_upper_bound = image.shape[0]  # 이미지의 끝 (480)

    right_discontinuities = 0
    last_valid_right_index = None
    discontinuous_window_found = False
    total_right_points = 0

    for window in range(max(right_nwindows, left_nwindows)):
        win_y_low_left = masked_image.shape[0] - (window + 1) * left_window_height
        win_y_high_left = masked_image.shape[0] - window * left_window_height

        win_y_low_right = masked_image.shape[0] - (window + 1) * right_window_height
        win_y_high_right = masked_image.shape[0] - window * right_window_height

        win_xleft_low = leftx_current - left_margin
        win_xleft_high = leftx_current + left_margin
        win_xright_low = rightx_current - right_margin
        win_xright_high = rightx_current + right_margin

        if win_y_high_left >= left_windows_lower_bound and win_y_low_left <= left_windows_upper_bound and apply_left_window:
            good_left_inds = ((nonzeroy >= win_y_low_left) & (nonzeroy < win_y_high_left) &
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
                left_lane_points_x.append(leftx_current)
                left_lane_points_y.append(int(np.mean(nonzeroy[good_left_inds])))

                cv2.rectangle(out_img, (win_xleft_low, win_y_low_left), (win_xleft_high, win_y_high_left), (255, 0, 0),
                              2)
                cv2.circle(out_img, (leftx_current, (win_y_low_left + win_y_high_left) // 2), 5, (0, 255, 0), -1)

        if apply_right_window:
            good_right_inds = ((nonzeroy >= win_y_low_right) & (nonzeroy < win_y_high_right) &
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
            if len(good_right_inds) > minpix:
                if discontinuous_window_found:
                    continue

                if last_valid_right_index is not None and window - last_valid_right_index > 1:
                    right_discontinuities += window - last_valid_right_index - 1

                if right_discontinuities > discontinuity_windows:
                    discontinuous_window_found = True
                    continue

                rightx_current = int(np.mean(nonzerox[good_right_inds]))
                right_lane_points_x.append(rightx_current)
                right_lane_points_y.append(int(np.mean(nonzeroy[good_right_inds])))
                last_valid_right_index = window

                total_right_points += len(good_right_inds)

                cv2.rectangle(out_img, (win_xright_low, win_y_low_right), (win_xright_high, win_y_high_right),
                              (0, 0, 255), 2)
                cv2.circle(out_img, (rightx_current, (win_y_low_right + win_y_high_right) // 2), 5, (0, 255, 0), -1)

    if total_right_points <= 5:
        apply_right_window = False

    return out_img, left_lane_points_x, left_lane_points_y, right_lane_points_x, right_lane_points_y


def calculate_path(right_fit, left_fit, x_la, real_shift_distance):
    if right_fit is not None:
        # 오른쪽 차선 함수로부터 경로 함수 계산
        R_prime_x_la = 3 * right_fit[0] * x_la ** 2 + 2 * right_fit[1] * x_la + right_fit[2]
        theta = np.arctan(R_prime_x_la)

        # 경로 함수 P(x)를 평행 이동으로 계산
        y_shift = +real_shift_distance * np.cos(theta)
        x_shift = -real_shift_distance * np.sin(theta)

        # 오른쪽 차선 함수에서의 y, x 평행 이동 적용
        ploty = np.linspace(0, 2, num=720)  # x축이 도로의 진행 방향이므로, 이게 ploty로 설정됨
        right_fit_x = right_fit[0] * ploty ** 3 + right_fit[1] * ploty ** 2 + right_fit[2] * ploty + right_fit[3]
        path_x = right_fit_x + y_shift
        path_y = ploty + x_shift

        return path_y, path_x  # X는 y 좌표, Y는 x 좌표로 반환

    elif left_fit is not None:
        # 왼쪽 차선 함수로부터 경로 함수 계산
        ploty = np.linspace(0, 2, num=720)
        left_fit_x = left_fit[0] * ploty + left_fit[1]
        path_x = left_fit_x - real_shift_distance  # 단순히 y축으로 이동
        path_y = ploty

        return path_y, path_x

    return None, None


def calculate_steering_angle(x_la, y_la):
    # Pure Pursuit을 사용한 조향각 계산
    steering_angle = np.arctan2(2 * L_m * y_la, x_la ** 2 + y_la ** 2)
    return np.degrees(steering_angle)


def calculate_steering_angle_from_slope(slope):
    # 차선의 기울기(슬로프)를 사용하여 조향각을 계산
    steering_angle_deg = np.degrees(np.arctan(slope))

    # 조향각이 -25도 이하로 작으면 -16도로 제한
    if steering_angle_deg <= -20:
        return -16
    return steering_angle_deg


def process_video_feed(video_path):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print("Cannot open the video.")
        return

    plt.ion()  # Interactive mode on
    fig, ax = plt.subplots()

    ax.set_xlim(0, 4)
    ax.set_ylim(-0.5, 0.5)

    x_la = 1.5  # Lookahead distance for path calculation in meters
    last_steering_angle = 0  # 마지막 계산된 조향각을 저장할 변수

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        bev = get_bird_eye_view(frame, (frame.shape[1], frame.shape[0]), fixed_points)
        out_img, left_lane_points_x, left_lane_points_y, right_lane_points_x, right_lane_points_y = sliding_window_demo(
            bev)

        if left_lane_points_x or right_lane_points_x:
            ax.clear()

            # 좌표 변환 및 1.341m 더하기 (figure의 X축 기준으로)
            left_lane_points_x_m = (275 - np.array(left_lane_points_x)) * x_m_per_pixel
            left_lane_points_y_m = (450 - np.array(left_lane_points_y)) * y_m_per_pixel + 1.341

            right_lane_points_x_m = (275 - np.array(right_lane_points_x)) * x_m_per_pixel
            right_lane_points_y_m = (450 - np.array(right_lane_points_y)) * y_m_per_pixel + 1.341

            ax.scatter(left_lane_points_y_m, left_lane_points_x_m, color='blue', label='Left Lane Points')
            ax.scatter(right_lane_points_y_m, right_lane_points_x_m, color='red', label='Right Lane Points')

            left_fit = None
            right_fit = None

            # Check for enough points to fit a polynomial
            if len(left_lane_points_x_m) >= 2:
                try:
                    left_fit = np.polyfit(left_lane_points_y_m, left_lane_points_x_m, 1)
                    left_fit_line = np.polyval(left_fit, left_lane_points_y_m)
                    ax.plot(left_lane_points_y_m, left_fit_line, color='blue', linewidth=2, label='Left Lane Fit')
                except np.linalg.LinAlgError as e:
                    print(f"LinAlgError for left fit: {e}")
            else:
                left_fit = None

            if len(right_lane_points_x_m) >= 4:
                try:
                    right_fit = np.polyfit(right_lane_points_y_m, right_lane_points_x_m, 3)
                    right_fit_line = np.polyval(right_fit, right_lane_points_y_m)
                    ax.plot(right_lane_points_y_m, right_fit_line, color='red', linewidth=2, label='Right Lane Fit')
                except np.linalg.LinAlgError as e:
                    print(f"LinAlgError for right fit: {e}")
            else:
                right_fit = None

            # 경로 계산 및 플롯 (차선 함수를 평행 이동하여 경로 함수 계산)
            path_y, path_x = calculate_path(right_fit, left_fit, x_la, real_shift_distance)
            if path_x is not None and path_y is not None:
                ax.plot(path_y, path_x, color='orange', linewidth=2, label='Path')

                if right_fit is not None:
                    # Calculate y_la corresponding to x_la using the path
                    path_fit = np.polyfit(path_y, path_x, 3)
                    y_la = np.polyval(path_fit, x_la)

                    # Plot the lookahead point
                    ax.plot(x_la, y_la, 'go', markersize=10, label='Look Ahead Point')

                    # Calculate steering angle using Pure Pursuit
                    steering_angle = calculate_steering_angle(x_la, y_la)
                elif left_fit is not None:
                    # Calculate steering angle directly from left lane slope
                    slope = left_fit[0]  # Slope of the left lane
                    steering_angle = calculate_steering_angle_from_slope(slope)
                else:
                    # 차선이 모두 감지되지 않은 경우 직전 조향각 유지
                    steering_angle = last_steering_angle

                print(f"Steering Angle: {steering_angle:.2f} degrees")
                last_steering_angle = steering_angle  # 마지막 조향각을 업데이트

        ax.set_xlabel("X Position (m)")
        ax.set_ylabel("Y Position (m)")
        ax.set_ylim(-0.6, 0.6)
        ax.set_xlim(0, 3.5)
        ax.legend(loc='lower right')
        plt.draw()
        plt.pause(0.01)

        cv2.imshow("Bird's Eye View with Lanes", out_img)
        cv2.imshow("Original Video", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    plt.ioff()
    plt.show()


if __name__ == "__main__":
    video_path = "trackrecord4_2x.mp4"
    process_video_feed(video_path)
