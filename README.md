# Detector-de-fugas-de-agua-distribuido-utilizando-Openthread-y-RTOS-
Este repositorio contiene el código de un prototipo domótico de detección de fugas de agua, desarrollado como Trabajo Final de Grado, basado en una arquitectura distribuida con dispositivos ESP32 y el protocolo Thread. Este repositorio está inacabado ya que faltan algunas mejoras en el código y documentación.

Resumen:

El sistema permite detectar la presencia de agua en una zona de interés y generar alertas locales (pantalla y buzzer) y remotas mediante integración con Home Assistant a través de MQTT.

Arquitectura del sistema:

El prototipo está compuesto por tres placas ESP32:

ESP32-H2 (sensor)
Encargada de la detección de agua mediante un sensor de gotas de agua . Implementa la pila completa OpenThread y envía alertas mediante UDP sobre Thread.

ESP32-H2 (RCP)
Actúa como Radio Co-Processor del Border Router, asegurando la comunicación IEEE 802.15.4 con la placa del sensor. Esta placa es necesaria para que el host pueda recibir el mensaje.

ESP32-S3 (host del Border Router)
Lider y gestor de la red Thread, reconstruye el mensaje original, activa los actuadores locales y publica alertas a un broker MQTT para su integración con Home Assistant.

La comunicación entre el RCP y el host del Border Router se realiza mediante el protocolo Spinel sobre UART.

Arquitectura basada en FreeRTOS, con separación de tareas y uso de colas para garantizar robustez.

Trabajo futuro:

Como línea de mejora, está prevista la integración de la capa Matter sobre el Border Router, esto permitiría que el sistema fuese también interoperable con dispositivos y plataformas domóticas comerciales.

Tecnologías utilizadas:

·OpenThread

·FreeRTOS

·IEEE 802.15.4

·IPv6 / UDP

·MQTT

·Home Assistant
