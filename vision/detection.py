import rclpy
from rclpy.node import Node # bo robimy skrypt ktory jest nodem (wezlem)
from sensor_msgs.msg import LaserScan   # wyciagamy stad ranges - odelglosci

# robimy node ktory bedzie podsluchiwac /scan z lidara

class ObstacleDetector(Node):
    
    #konstruktor
    def __init__(self): 
        super().__init__('obstacle_detector')   # wywolujemy konstruktor klasy nadrzednej - Node czyli jak zrobimy w konsoli ros2 node list to sie pojawi!
        self.subscription = self.create_subscription(
            LaserScan,      # typ spodziewanej wiadomoci
            '/scan',        # topic ktory sledzumy
            self.listener_callback, # wwylouje sie zawsze za kazdym razem jak przyjda nowe dane z lidaru
            10              # quality of service - ponoc 10 to standard wiec zostawiam ale nie wiem co to zmienia
        )

    # funkcja
    def listener_callback(self, msg):   # msg to obiekt typu LaserScan
        
        ranges = msg.ranges     # lista dlugosci z pomiarow
        n = len(ranges)         # liczba pomiarow w skanie jendym

        front = ranges[n//2]            # n//2 to srodkowy pomiar // to dzielenie calkowite wiec nie ma jakis gowien po przecinkach
        left = min(ranges[n//2:n-1])    # tablica od srodka jak wyzej i w lewo czyli do tylu to -1
        right = min(ranges[0:n//2])     # reszta co jest dopoki nie polowa 

        if front != float('inf'):   # inf = infinity
            front = front
        else:
            front = 999

        if left != float('inf'):
            left = left
        else:
            left = 999

        if right != float('inf'):
            right = right
        else:
            right = 999

        # self.get_logger().info(...) to jak print() ale w ros2
        # ponizej 1m daje info jak blisko przeszkoda
        if front < 1.0: 
            self.get_logger().info(f"Przeszkoda z przodu w odległości {front:.2f} m")   #float z 2 miejscami po przecinku
        if left < 1.0:
            self.get_logger().info(f"Przeszkoda z lewej w odległości {left:.2f} m")
        if right < 1.0:
            self.get_logger().info(f"Przeszkoda z prawej w odległości {right:.2f} m")
    

def main(args=None):
    rclpy.init(args=args)           #inicjalizacja ros2
    detector = ObstacleDetector()   #tworzymy obiekt node
    rclpy.spin(detector)            #laser sie kreci i skanuje
    detector.destroy_node()         #czysci sie jak wylacza, ponoc trzeba dbac o pamiec :)
    rclpy.shutdown()                #zamyka ros2

if __name__ == '__main__':          #let the game begin ale tylko bezposrednio
    main()


