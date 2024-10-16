#include <WiFi.h>
#include <PubSubClient.h>

// Clase SensorUltrasonico para encapsular la funcionalidad del sensor
class SensorUltrasonico {
  private:
    int triggerPin;
    int echoPin;
    
  public:
    SensorUltrasonico(int trigger, int echo) : triggerPin(trigger), echoPin(echo) {
      pinMode(triggerPin, OUTPUT);
      pinMode(echoPin, INPUT);
    }

    // Método para leer la distancia
    long leerDistancia() {
      digitalWrite(triggerPin, LOW);
      delayMicroseconds(4);
      digitalWrite(triggerPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(triggerPin, LOW);
      return pulseIn(echoPin, HIGH);
    }
};

// Clase Led para encapsular la funcionalidad de los LEDs
class Led {
  private:
    int pin;

  public:
    Led(int ledPin) : pin(ledPin) {
      pinMode(pin, OUTPUT);
    }

    void encender() {
      digitalWrite(pin, HIGH);
    }

    void apagar() {
      digitalWrite(pin, LOW);
    }
};

// Clase ControladorMQTT para manejar la conexión WiFi y el control de MQTT
class ControladorMQTT {
  private:
    const char* ssid;
    const char* password;
    const char* mqttServer;
    const int mqttPort;
    const char* mqttUser;
    const char* mqttPassword;
    const char* mqttLedTopic;
    const char* mqttDistanceTopic;
    bool autoControl;
    WiFiClient espClient;
    PubSubClient client;
    Led redLed;
    Led whiteLed;
    SensorUltrasonico sensor;
    unsigned long previousMillis;  // Para controlar el temporizador

  public:
    // Constructor
    ControladorMQTT(const char* wifiSSID, const char* wifiPassword, const char* server, int port, const char* user, const char* pass, const char* ledTopic, const char* distanceTopic, Led& rLed, Led& wLed, SensorUltrasonico& sensorProx)
      : ssid(wifiSSID), password(wifiPassword), mqttServer(server), mqttPort(port), mqttUser(user), mqttPassword(pass), mqttLedTopic(ledTopic), mqttDistanceTopic(distanceTopic), client(espClient), redLed(rLed), whiteLed(wLed), sensor(sensorProx), autoControl(true), previousMillis(0) {}

    void iniciarWiFi() {
      Serial.begin(115200);
      WiFi.begin(ssid, password);
      while (WiFi.status() != WL_CONNECTED) {
        delay(2000);
        Serial.println("Conectando a WiFi...");
      }
      Serial.println("Conectado a la red WiFi");
    }

    void configurarMQTT() {
      client.setServer(mqttServer, mqttPort);
      client.setCallback([this](char* topic, byte* payload, unsigned int length) { this->callback(topic, payload, length); });
      conectarMQTT();
    }

    void conectarMQTT() {
      while (!client.connected()) {
        Serial.println("Conectando al broker MQTT...");
        if (client.connect("ArduinoClient", mqttUser, mqttPassword)) {
          Serial.println("Conectado al broker MQTT");
          client.subscribe(mqttLedTopic);
        } else {
          Serial.print("Fallo de conexión. Estado = ");
          Serial.println(client.state());
          delay(2000);
        }
      }
    }

    void gestionarMensajes() {
      if (!client.connected()) {
        conectarMQTT();
      }
      client.loop();
    }

    void callback(char* topic, byte* payload, unsigned int length) {
      String message;
      for (int i = 0; i < length; i++) {
        message += (char)payload[i];
      }
      message.trim();
      Serial.print("Mensaje recibido: ");
      Serial.println(message);
      Serial.print("Tópico: ");
      Serial.println(topic);

      if (String(topic) == mqttLedTopic) {
        gestionarControlManual(message);
      }
    }

    void gestionarControlManual(String message) {
      if (message.equalsIgnoreCase("RED_LED on")) {
        redLed.encender();
        autoControl = false;
      } else if (message.equalsIgnoreCase("RED_LED off")) {
        redLed.apagar();
        autoControl = false;
      } else if (message.equalsIgnoreCase("WHITE_LED on")) {
        whiteLed.encender();
        autoControl = false;
      } else if (message.equalsIgnoreCase("WHITE_LED off")) {
        whiteLed.apagar();
        autoControl = false;
      } else if (message.equalsIgnoreCase("autoControl on")) {
        autoControl = true;
      } else if (message.equalsIgnoreCase("autoControl off")) {
        autoControl = false;
      }
    }

    // Método para gestionar el control automático basado en la distancia
    void gestionarControlAutomatico() {
      if (autoControl) {
        int distance_cm = round(0.01723 * sensor.leerDistancia());
        Serial.print("Distancia medida: ");
        Serial.println(distance_cm);
        client.publish(mqttDistanceTopic, String(distance_cm).c_str());

        if (distance_cm >= 2 && distance_cm <= 20) {
          redLed.encender();
          whiteLed.apagar();
        } else if (distance_cm > 20 && distance_cm <= 60) {
          whiteLed.encender();
          redLed.apagar();
        } else {
          redLed.apagar();
          whiteLed.apagar();
        }
      }
    }

    // Método para gestionar el temporizador y enviar la distancia automáticamente cada segundo
    void gestionarTemporizador() {
      unsigned long currentMillis = millis();
      
      if (autoControl && (currentMillis - previousMillis >= 1000)) {  // Cada segundo
        previousMillis = currentMillis;
        gestionarControlAutomatico();  // Realizar medición y enviar la distancia
      }
    }
};

#define trigger_pin 27   // Pin del trigger del sensor de proximidad
#define echo_pin 26      // Pin del echo del sensor de proximidad
#define redLedPin 12    // LED rojo
#define whiteLedPin 14  // LED blanco

Led redLed(redLedPin);
Led whiteLed(whiteLedPin);
SensorUltrasonico sensor(trigger_pin, echo_pin);
ControladorMQTT controladorMQTT("Pixel 6 Pro de pollo", "3755484011", "broker.hivemq.com", 1883, "", "", "casa/led", "casa/distancia", redLed, whiteLed, sensor);

void setup() {
  controladorMQTT.iniciarWiFi();
  controladorMQTT.configurarMQTT();
}

void loop() {
  controladorMQTT.gestionarMensajes();
  controladorMQTT.gestionarTemporizador();  // Comienza a enviar la distancia automáticamente cada segundo
}

