import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # ou qualquer tipo de mensagem que você está usando no ROS2
import paho.mqtt.client as mqtt
from functools import partial

class ROS2ToMQTTBridge(Node):

    def __init__(self):
        super().__init__('ros2_to_mqtt_bridge')
        self.topics_to_convert = {
            '/b1/defect_found': '/b1/defect_found/mqtt',
            '/b2/defect_found': '/b2/defect_found/mqtt',
            '/b3/defect_found': '/b3/defect_found/mqtt',
        }
        
        # Configurações do MQTT
        self.mqtt_broker = 'localhost'
        self.mqtt_port = 1883
        self.mqtt_user = 'corkai'
        self.password = 'corkai123'
        
        # Iniciando cliente MQTT
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_disconnect = self.on_disconnect
        self.mqtt_client.username_pw_set(username=self.mqtt_user, password=self.password)
        self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
        self.mqtt_client.loop_start()  # Inicie o loop do cliente MQTT
        
        # Iniciando inscrição nos tópicos ROS2
        for ros_topic, mqtt_topic in self.topics_to_convert.items():
            self.create_subscription(
                String,  # Tipo de mensagem ROS2
                ros_topic,
                partial(self.ros2_callback, mqtt_topic=mqtt_topic),  # Passando o tópico MQTT como argumento
                10)

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info('Conectado ao broker MQTT com sucesso')
        else:
            self.get_logger().error(f'Falha ao conectar ao broker MQTT, código de retorno: {rc}')

    def on_disconnect(self, client, userdata, rc):
        self.get_logger().info('Desconectado do broker MQTT')

    def ros2_callback(self, msg, mqtt_topic):
        self.get_logger().info(f'Recebido mensagem de ROS2: "{msg.data}" no tópico {mqtt_topic}')
        # Publicando a mensagem recebida no tópico MQTT correspondente
        self.mqtt_client.publish(mqtt_topic, msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = ROS2ToMQTTBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mqtt_client.loop_stop()  # Parar loop MQTT
        node.mqtt_client.disconnect()  # Desconectar cliente MQTT
        node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()