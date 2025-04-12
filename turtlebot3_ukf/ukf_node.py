import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped  # Robot pozisyonu için mesaj tipi
from sensor_msgs.msg import Imu  # IMU verisi için mesaj tipi
from nav_msgs.msg import Odometry  # Odometri için
from tf2_ros import TransformListener, Buffer  # Koordinat dönüşümleri için
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints
import numpy as np


class UKFNode(Node):
    def __init__(self):
        super().__init__('ukf_node')  # UKF Node'u başlatıyoruz

        self.x = np.array([0., 0., 0.])  # Başlangıç tahmini: x, y, theta (açı)
        self.P = np.eye(3)  # Hata kovaryans matrisi
        self.dt = 0.1  # Zaman adımı (0.1 saniye)

        # Sigma noktaları ve UKF algoritmasını başlatıyoruz
        self.sigma_points = MerweScaledSigmaPoints(n=3, alpha=0.1, beta=2, kappa=0.)
        self.ukf = UKF(
            dim_x=3,
            dim_z=2,
            fx=self.state_transition,
            hx=self.measurement_function,
            points=self.sigma_points,
            dt=self.dt  # Burada dt parametresini ekliyoruz
        )


        # IMU verisini almak için subscriber oluşturuyoruz
        self.imu_subscriber = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10
        )

        # Odometri verisini almak için subscriber oluşturuyoruz
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # Robotun pozisyonunu yayınlamak için publisher oluşturuyoruz
        self.pose_publisher = self.create_publisher(PoseStamped, '/ukf_node', 10)

        self.get_logger().info("UKF Node başlatıldı")  # Node başlatıldığını logluyoruz

    def state_transition(self, state, dt):
        """ Durum geçiş fonksiyonu """
        x, y, theta = state
        v_x = 0.1  # Hız (örnek)
        v_y = 0.0  # Yalnızca x yönünde hareket
        theta_dot = 0.05  # Açı değişimi (dönme hızı)

        # Geçiş fonksiyonu (robotun yeni durumu)
        x_new = x + v_x * dt * np.cos(theta)
        y_new = y + v_y * dt * np.sin(theta)
        theta_new = theta + theta_dot * dt

        return np.array([x_new, y_new, theta_new])

    def measurement_function(self, state):
        """ Pozisyon ölçüm fonksiyonu (x, y) """
        return np.array([state[0], state[1]])

    def imu_callback(self, msg):
        """ IMU verisini işliyoruz """
        yaw = msg.orientation.z  # Yaw açısını alıyoruz
        self.get_logger().info(f"IMU Yaw: {yaw}")  # Yaw'ı logluyoruz

        # IMU verisini filtreye ekliyoruz
        self.ukf.predict()  # Tahmin yapıyoruz
        self.ukf.update(np.array([yaw, 0]))  # Yaw'ı ve 0'ı ekliyoruz (mesafe burada eksik)

    def odom_callback(self, msg):
        """ Odometri verisini işliyoruz """
        x = msg.pose.pose.position.x  # X pozisyonu
        y = msg.pose.pose.position.y  # Y pozisyonu
        self.get_logger().info(f"Odometri Pozisyon: x={x}, y={y}")  # Bu veriyi logluyoruz

        # Odometri verisini filtreye ekliyoruz
        self.ukf.predict()  # Tahmin yapıyoruz
        self.ukf.update(np.array([x, y]))  # X ve Y'yi ekliyoruz

    def publish_node(self):
        """ Robotun pozisyonunu yayımlıyoruz """
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"  # Pozisyonu 'map' koordinatına göre yayımlıyoruz

        # Şu an filtrelenmiş pozisyonu (X, Y) gönderiyoruz
        pose_msg.pose.position.x = self.ukf.x[0]  # X pozisyonunu filtreyle tahmin ediyoruz
        pose_msg.pose.position.y = self.ukf.x[1]  # Y pozisyonunu filtreyle tahmin ediyoruz
        pose_msg.pose.position.z = 0.0  # Z düzeyinde pozisyon (yere paralel)

        # Bu pozisyonu /ukf_pose topic'ine gönderiyoruz
        self.pose_publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)  # ROS2'yi başlatıyoruz
    node = UKFNode()  # UKF node'unu başlatıyoruz

    rclpy.spin(node)  # Node'u sürekli çalıştırıyoruz

    node.destroy_node()  # Node'u yok ediyoruz
    rclpy.shutdown()  # ROS2'yi kapatıyoruz

if __name__ == '__main__':
    main()  # Ana fonksiyonu başlatıyoruz
