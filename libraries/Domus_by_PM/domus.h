// library file for domus sketches
#include <Ethernet.h>           // Ethernet.h library
#include "PubSubClient.h"       //PubSubClient.h Library from Knolleary, must be adapted: #define MQTT_MAX_PACKET_SIZE 512
#include "ArduinoJson.h"
#define BUFFERSIZE 512              // default 100
#define MQTT_MAX_PACKET_SIZE 512
#define DEBOUNCE_DELAY 150  // debounce delay for buttons
#define LONGPRESS_TIME 450  // time for press to be detected as 'long'
StaticJsonDocument<512> doc;
EthernetClient ethClient;
PubSubClient mqttClient;
char messageBuffer[BUFFERSIZE];
char topicBuffer[BUFFERSIZE];
String ip = "";
bool startsend = HIGH;// flag for sending at startup
// Vul hier het interval in waarmee gegevens worden verstuurd op MQTT
#define PUBLISH_DELAY 10000 // that is 10 second interval
long lastPublishTime;
// Vul hier het interval in waarmee alle statussen worden verstuurd op MQTT
#define REPORT_DELAY 60000 // that is 60 seconds interval
long lastReportTime;
#if defined (DEBUG)
bool debug = true;
#else
bool debug = false;
#endif
void(* resetFunc) (void) = 0; //declare reset function @ address 0
void ShowDebug(String tekst) {
  if (debug) {
    Serial.println(tekst);
  }
}
String mac2String(byte ar[]) {
  String s;
  for (byte i = 0; i < 6; ++i)
  {
    char buf[3];
    sprintf(buf, "%02X", ar[i]);
    s += buf;
    if (i < 5) s += ':';
  }
  return s;
}
void report_state_relay() {
  bool relaytest;
  if (NumberOfRelays > 0) {
    doc.clear();
    for (int i = 0; i < NumberOfRelays; i++) {
      if (RelayPins[i] < 100) {
        relaytest = (digitalRead(RelayPins[i]) == RelayInitialState[i]);
      }
#if defined(MCP_present)
      else {
        relaytest = (mcp.digitalRead(RelayPins[i] - 100) == RelayInitialState[i]);
      }
#endif
      if (relaytest) {
        doc["POWER" + String(i)] = "off";
      }
      else {
        doc["POWER" + String(i)] = "on";
      }
      serializeJson(doc, messageBuffer);
    }
    ShowDebug("Sending MQTT state for relays...");
    ShowDebug(messageBuffer);
    mqttClient.publish(state_topic_relays, messageBuffer);
  }
}
void report_state_light(int index) {
  StaticJsonDocument<256> outputdoc;
  outputdoc.clear();
  if (LightBrightness[index]) {
    if (LightValue[index] > 0) {
      outputdoc["state"] = "ON";
    }
    else {
      outputdoc["state"] = "OFF";
    }
    outputdoc["brightness"] = LightValue[index];
  }
  else if (digitalRead(LightPins[index]) == LightInitialState[index]) {
    outputdoc["state"] = "OFF";
  }
  else {
    outputdoc["state"] = "ON";
  }
  serializeJson(outputdoc, messageBuffer);
  ShowDebug("Sending MQTT state for lights...");
  ShowDebug(messageBuffer);
  mqttClient.publish((state_topic_lights + String(index + 1)).c_str(), messageBuffer);
}
void report_state_cover() {
  if (NumberOfCovers > 0) {
    StaticJsonDocument<256> outputdoc;
    outputdoc.clear();
    for (byte i = 0; i < NumberOfCovers ; i++) {
      if (CoverState[i] == 0) {
        outputdoc["COVER" + String(i + 1)] = "open";
      }
      else if (CoverState[i] == 1) {
        outputdoc["COVER" + String(i + 1)] = "opening";
      }
      else if (CoverState[i] == 2) {
        outputdoc["COVER" + String(i + 1)] = "closed";
      }
      else if (CoverState[i] == 3) {
        outputdoc["COVER" + String(i + 1)] = "closing";
      }
      else if (CoverState[i] == 4) {
        outputdoc["COVER" + String(i + 1)] = "stopped";
      }
      outputdoc["POSITION" + String(i + 1)] = CoverPos[i];
    }
    serializeJson(outputdoc, messageBuffer);
    ShowDebug("Sending MQTT state for covers...");
    ShowDebug(messageBuffer);
    mqttClient.publish(state_topic_covers, messageBuffer);
  }
}
void report_state_lock() {
  if (NumberOfLocks > 0) {
    doc.clear();
    for (byte i = 0; i < NumberOfLocks ; i++) {
      if (LockState[i] == 0) {
        doc["LOCK" + String(i + 1)] = "UNLOCKED";
      }
      else {
        doc["LOCK" + String(i + 1)] = "LOCKED";
      }
    }
    serializeJson(doc, messageBuffer);
    ShowDebug("Sending MQTT state for locks...");
    ShowDebug(messageBuffer);
    mqttClient.publish(state_topic_locks, messageBuffer);
  }
}
void report_state_pir() {
  if (NumberOfPirs > 0) {
    doc.clear();
    for (byte i = 0; i < NumberOfPirs ; i++) {
      if (PirState[i] == 0) {
        doc["PIR" + String(i + 1)] = "OFF";
      }
      else {
        doc["PIR" + String(i + 1)] = "ON";
      }
    }
    serializeJson(doc, messageBuffer);
    ShowDebug("Sending MQTT state for pirs...");
    ShowDebug(messageBuffer);
    mqttClient.publish(state_topic_pirs, messageBuffer);
  }
}
void report_state() {
  // send data for relays
  report_state_relay();
  // send data for lights
  if (NumberOfLights > 0) {
    for (int index = 0; index < NumberOfLights ; index++) {
      report_state_light(index);
    }
  }
  // send data for covers
  report_state_cover();
  // send data for locks
  report_state_lock();
  // end send state data for MQTT discovery
}
void SetLightState(int light, String state) {
  if (state == "ON") {
    ShowDebug("Set relay " + String(LightPins[light]) + " on.");
    digitalWrite(LightPins[light], !LightInitialState[light]);
    report_state_light(light);
  }
  else if (state == "OFF") {
    ShowDebug("Set relay " + String(LightPins[light]) + " off.");
    digitalWrite(LightPins[light], LightInitialState[light]);
    LightValue[light] = 0;
    report_state_light(light);
  }
  else {
    ShowDebug("Setting pwm value " + String(LightValue[light]) + "on pin " + String(LightPins[light]));
    if (LightInitialState[light]) {
      analogWrite(LightPins[light], 255 - LightValue[light]);
    }
    else {
      analogWrite(LightPins[light], LightValue[light]);
    }
    report_state_light(light);
  }
}
void ProcessPulseRelays(int PulseRelayId) {
  // Process the timers of the pulse relays and see if we have to close them.
  if (digitalRead(PulseRelayPins[PulseRelayId]) == !PulseRelayInitialStates[PulseRelayId])
  {
    if ((millis() - PulseActivityTimes[PulseRelayId]) > PulseRelayTimes[PulseRelayId])
    {
      ShowDebug("Disabling pulse relay" + String(PulseRelayId) + ".");
      ShowDebug(String(PulseActivityTimes[PulseRelayId]));
      String messageString = "P" + String(PulseRelayId) + "0";
      messageString.toCharArray(messageBuffer, messageString.length() + 1);
      mqttClient.publish(topic_out_pulse, messageBuffer);
      digitalWrite(PulseRelayPins[PulseRelayId], PulseRelayInitialStates[PulseRelayId]);

      // Update end position of covers
      for (int i = 0 ; i < NumberOfCovers ; i++) {
        ShowDebug("Checking status of cover " + String(i));
        ShowDebug("CoverPulse: " + String(CoverPulse[i]) + " - " + String(PulseRelayId));
        if (CoverPulse[i] == PulseRelayId) {
          if (CoverState[i] == 1) {   // 1 = opening
            CoverPos[i] = 100; // positon 100 = open
            CoverState[i] = 0;  // 0 = open
          }
          else {
            CoverPos[i] = 0;    // position 0 = closed
            CoverState[i] = 2;  // 2 = closed
          }
          ShowDebug("Setting cover position:");
          ShowDebug(String(CoverPos[i]));
        }
      } // end update covers
      // Update lock status
      for (int i = 0 ; i < NumberOfLocks ; i++) {
        if (LockPulse[i] == PulseRelayId) {
          if (LockState[i] == 0) {   // 0 = unlocked
            LockState[i] = 1;
            report_state_lock();
          }
        }
      } // end update lock status
    }
    else { // Update current position of covers
      int LastPosition;
      for (int i = 0 ; i < NumberOfCovers ; i++) {
        if (CoverPulse[i] == PulseRelayId) {
          LastPosition = CoverPos[i];
          int progress = int(100 * (millis() - PulseActivityTimes[PulseRelayId]) / PulseRelayTimes[PulseRelayId]);
          if (CoverState[i] == 1) {     // 1 = opening, position goes from current position to 100
            CoverPos[i] = CoverStart[i] + progress;
          } else if (CoverState[i] == 3) {     // 3 = closing, position goes from current position to 0
            CoverPos[i] = CoverStart[i] - progress;
          }
          if (LastPosition != CoverPos[i]) {
            report_state_cover();
          }
        }
      } // end update covers
    }
  }
}
void processButtonDigital( int buttonId ) {
  if ( !digitalRead(ButtonPins[buttonId]) ) // Input pulled low to GND. Button pressed.
  {
    if ( !lastButtonStates[buttonId] )  // The button was previously un-pressed
    {
      if ((millis() - lastActivityTimes[buttonId]) > DEBOUNCE_DELAY) // Proceed if we haven't seen a recent event on this button
      {
        lastActivityTimes[buttonId] = millis();
      }
    }
    else if ((millis() - lastActivityTimes[buttonId] > LONGPRESS_TIME) && (!LongPressActive[buttonId]))// Button long press
    {
      LongPressActive[buttonId] = true;
      ShowDebug( "Button" + String(buttonId) + " long pressed" );
      String messageString = "Button" + String(buttonId) + "_long";
      messageString.toCharArray(messageBuffer, messageString.length() + 1);
      mqttClient.publish(topic_out, messageBuffer);
    }
    lastButtonStates[buttonId] = HIGH;
  }
  else {
    if (lastButtonStates[buttonId] == HIGH) {
      if (LongPressActive[buttonId]) {
        LongPressActive[buttonId] = false;
      } else {
        if ((millis() - lastActivityTimes[buttonId]) > DEBOUNCE_DELAY) // Proceed if we haven't seen a recent event on this button
        {
          ShowDebug( "Button" + String(buttonId) + " pressed" );
          String messageString = "Button" + String(buttonId);
          messageString.toCharArray(messageBuffer, messageString.length() + 1);
          mqttClient.publish(topic_out, messageBuffer);
        }
      }
      lastButtonStates[buttonId] = LOW;
    }
  }
}
void check_pir(byte pirid) {
  // ...read out the PIR sensors...
  if (digitalRead(PirSensors[pirid]) == !lastPirStates[pirid])
  {
    PirLastActivityTimes[pirid] = millis();
    lastPirStates[pirid] = digitalRead(PirSensors[pirid]);
  }
  if (digitalRead(PirSensors[pirid]) == !PirInitialState[pirid]) 
  {
    if (!PreviousDetects[pirid]) {
      if ((millis() - PirLastActivityTimes[pirid]) > PirDebounce[pirid])
      { 
          ShowDebug("Pir " + String(pirid) + " on.");
          PreviousDetects[pirid] = true;
          PirState[pirid] = 1;
          report_state_pir();
      }
    }
  }
  else {
    if (PreviousDetects[pirid]) {
      ShowDebug("Pir " + String(pirid) + " off.");
      PirState[pirid] = 0;
      report_state_pir();
    }
    PreviousDetects[pirid] = false;
  }
}
#if defined(MQ7_present)
float raw_value_to_CO_ppm(float value) {
  float reference_resistor_kOhm = 10.0;

  float sensor_reading_100_ppm_CO = -1;
  float sensor_reading_clean_air = 675;

  float sensor_100ppm_CO_resistance_kOhm;
  float sensor_base_resistance_kOhm;

  if (value < 1) return -1; //wrong input value
  sensor_base_resistance_kOhm = reference_resistor_kOhm * 1023 / sensor_reading_clean_air - reference_resistor_kOhm;
  if (sensor_reading_100_ppm_CO > 0)
  {
    sensor_100ppm_CO_resistance_kOhm = reference_resistor_kOhm * 1023 / sensor_reading_100_ppm_CO - reference_resistor_kOhm;
  }
  else
  {
    sensor_100ppm_CO_resistance_kOhm = sensor_base_resistance_kOhm * 0.25;
    //UPDATED: when checked on a CO meter, it seems that sensor corresponds to
    //the datasheet pretty well
  }
  float sensor_R_kOhm = reference_resistor_kOhm * 1023 / value - reference_resistor_kOhm;
  float R_relation = sensor_100ppm_CO_resistance_kOhm / sensor_R_kOhm;
  float CO_ppm = 134 * R_relation - 35;
  if (CO_ppm < 0) CO_ppm = 0;
  return CO_ppm;
}
#endif
void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    ShowDebug("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect(CLIENT_ID)) {
      ShowDebug("connected");
      // Once connected, publish an announcement...
      mqttClient.publish(topic_out, ip.c_str());
      mqttClient.publish(topic_out, CLIENT_ID);
      // ... and resubscribe
      mqttClient.subscribe(topic_in);
    } else {
      ShowDebug("MQTT connection failed, rc=" + String(mqttClient.state()));
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void sendMessage(String m, char* topic) {
  m.toCharArray(messageBuffer, m.length() + 1);
  mqttClient.publish(topic, messageBuffer);
}
float CheckIsNan(float value, float defaultvalue) {
  if (isnan(value)) value = defaultvalue;
  return value;
}
void sendData() {
  float t, h, hic;
  doc.clear();
  for (int i = 0; i < NumberOfSensors; i++) {
    if (SensorTypes[i] == "TIME") {
      doc["sensor" + String(i + 1)] = millis() / 1000;
    }
#if defined(DHT_present)
    else if (SensorTypes[i] == "DHT-T") {
      t = CheckIsNan(dht.readTemperature(), 0);
      doc["sensor" + String(i + 1)] = t;
    }
    else if (SensorTypes[i] == "DHT-H") {
      h = CheckIsNan(dht.readHumidity(), 0);
      doc["sensor" + String(i + 1)] = h;
    }
    else if (SensorTypes[i] == "DHT-I") {
      hic = CheckIsNan(dht.computeHeatIndex(t, h, false), -99);
      doc["sensor" + String(i + 1)] = hic;
    }
#endif
#if defined(LDR_present)
    else if (SensorTypes[i] == "LDR") {
      doc["sensor" + String(i + 1)] = map(analogRead(LightSensor), 0, 1023, 0, 100);
    }
#endif
#if defined(DS18B20_present)
    else if (SensorTypes[i] == "DS18B20") {
      t = sensors.getTempCByIndex(0);
      if (t < -50) { // fix for anomalous readings
        t = last_temp;
      }
      else if (t > 50) { // fix for anomalous readings
        t = last_temp;
      }
      else {
        last_temp = t;
      }
      doc["sensor" + String(i + 1)] = t;
      sensors.requestTemperatures();
    }
#endif
#if defined(BMP_present)
    else if (SensorTypes[i] == "BMP-T") {
      t = bmp.readTemperature();
      doc["sensor" + String(i + 1)] = t;
    }
    else if (SensorTypes[i] == "BMP-P") {
      t = bmp.readPressure() / 100;
      doc["sensor" + String(i + 1)] = t;
    }
#endif
#if defined(MQ_present)
    else if (SensorTypes[i] == "MQ2") {
      doc["sensor" + String(i + 1)] = String(map(analogRead(SmokeSensor), 0, 1023, 0, 100));
    }
#endif
#if defined(MQ7_present)
    else if (SensorTypes[i] == "MQ7") {
      doc["sensor" + String(i + 1)] = raw_value_to_CO_ppm(co_value);
    }
#endif
  }
  serializeJson(doc, messageBuffer);
  ShowDebug("Sending MQTT state for sensors...");
  ShowDebug(messageBuffer);
  mqttClient.publish(state_topic_sensors, messageBuffer);

}
void OpenCover(int Cover) {
  int PulseRelayPort = CoverPulse[Cover];
  ShowDebug("Pulse relay index = " + String(PulseRelayPort));
  ShowDebug("Pulse relay port = " + String(PulseRelayPins[PulseRelayPort]));
  ShowDebug("Status port = " + String(digitalRead(PulseRelayPins[PulseRelayPort])));
  // Check if another command is already running
  if (digitalRead(PulseRelayPins[PulseRelayPort]) == PulseRelayInitialStates[PulseRelayPort]) {
    ShowDebug("Opening cover number " + String(Cover + 1));
    // Set direction relay
    digitalWrite(RelayPins[CoverDir[Cover]], !RelayInitialState[CoverDir[Cover]]);
    // Set opening pulse
    digitalWrite(PulseRelayPins[PulseRelayPort], !PulseRelayInitialStates[PulseRelayPort]);
    String messageString = "P" + String(PulseRelayPort) + "1";
    messageString.toCharArray(messageBuffer, messageString.length() + 1);
    mqttClient.publish(topic_out_pulse, messageBuffer);
    ShowDebug("Cover: Starting pulse time on " + String(millis()));
    PulseActivityTimes[PulseRelayPort] = millis();
    // report state of screen
    CoverState[Cover] = 1; // 1 is opening
    CoverStart[Cover] = CoverPos[Cover];
    report_state_cover();
  }
  else {
    // Commmand given while cover moving, Stop pulse
    ShowDebug("Stopping cover number " + String(Cover));
    digitalWrite(PulseRelayPins[PulseRelayPort], PulseRelayInitialStates[PulseRelayPort]);
    String messageString = "P" + String(PulseRelayPort) + "0";
    messageString.toCharArray(messageBuffer, messageString.length() + 1);
    mqttClient.publish(topic_out_pulse, messageBuffer);
  }
}
void CloseCover(int Cover) {
  int PulseRelayPort = CoverPulse[Cover];
  // Check if another command is already running
  if (digitalRead(PulseRelayPins[PulseRelayPort]) == PulseRelayInitialStates[PulseRelayPort]) {
    ShowDebug("Closing cover number " + String(Cover + 1));
    // Set direction relay
    digitalWrite(RelayPins[CoverDir[Cover]], RelayInitialState[CoverDir[Cover]]);
    // Set opening pulse
    digitalWrite(PulseRelayPins[PulseRelayPort], !PulseRelayInitialStates[PulseRelayPort]);
    String messageString = "P" + String(PulseRelayPort) + "1";
    messageString.toCharArray(messageBuffer, messageString.length() + 1);
    mqttClient.publish(topic_out_pulse, messageBuffer);
    ShowDebug("Cover: Starting pulse time on " + String(millis()));
    PulseActivityTimes[PulseRelayPort] = millis();
    // report state of screen
    CoverState[Cover] = 3;  // 3 is closing
    CoverStart[Cover] = CoverPos[Cover];
    report_state_cover();
  }
  else {
    ShowDebug("Stopping cover number " + String(Cover + 1));
    // Commmand given while cover moving, Stop pulse
    digitalWrite(PulseRelayPins[PulseRelayPort], PulseRelayInitialStates[PulseRelayPort]);
    String messageString = "P" + String(PulseRelayPort) + "0";
    messageString.toCharArray(messageBuffer, messageString.length() + 1);
    mqttClient.publish(topic_out_pulse, messageBuffer);
  }
}
void StopCover(int Cover) {
  if ((CoverState[Cover] & 1) == 1) {
    int PulseRelayPort = CoverPulse[Cover];
    // Stop pulse
    ShowDebug("Stop cover number " + String(Cover + 1));
    digitalWrite(PulseRelayPins[PulseRelayPort], PulseRelayInitialStates[PulseRelayPort]);
    String messageString = "P" + String(PulseRelayPort) + "0";
    messageString.toCharArray(messageBuffer, messageString.length() + 1);
    mqttClient.publish(topic_out_pulse, messageBuffer);
  }
  if (CoverPos[Cover] <= 0) {
    CoverPos[Cover] = 0;
    CoverState[Cover] = 2;
  }
  else if (CoverPos[Cover] >= 100) {
    CoverPos[Cover] = 100;
    CoverState[Cover] = 0;
  }
  else {
    CoverState[Cover] = 4;
  }
}
void SetCoverPosition(int cover, int position) {
  CoverSetPos[cover] = position;
  if (CoverPos[cover] < position) {
    ShowDebug("Cover needs to open");
    if (position == 100) {
      CoverSetPos[cover] = 120;
    }
    CoverState[cover] = 1; // opening
    OpenCover(cover);
  }
  else {
    ShowDebug("Cover needs to close");
    if (position == 0) {
      CoverSetPos[cover] = -20;
    }
    CoverState[cover] = 3; // closing
    CloseCover(cover);
  }
}
void ProcessCovers(int cover) {
  if (CoverPos[cover] == CoverSetPos[cover]) {
    StopCover(cover);
  }
}
void callback(char* topic, byte * payload, byte length) {
  char msgBuffer[BUFFERSIZE];
  payload[length] = '\0'; // terminate string with 0
  String strPayload = String((char*)payload);  // convert to string
  ShowDebug("Message arrived");
  ShowDebug(topic);
  ShowDebug(strPayload);

  byte RelayPort;
  byte RelayValue;

  if (String(topic).indexOf(cmd_topic_lights) >= 0) {
    doc.clear();
    deserializeJson(doc, strPayload);
    int index = (topic[strlen(topic) - 1]) - '0';
    if (LightBrightness[index - 1] && doc.containsKey("brightness")) {
      LightValue[index - 1] = doc["brightness"];
      SetLightState(index - 1, doc["brightness"]);
    }
    else {
      if (LightBrightness[index - 1]) {
        LightValue[index - 1] = 255;
      }
      SetLightState(index - 1, doc["state"]);
    }
  }
  else if (strPayload[0] == '{') {
    // json message
    deserializeJson(doc, strPayload);
    for (int i = 0; i < NumberOfCovers; i++) {
      String keyword = "POSITION" + String(i + 1);
      ShowDebug("Keyword: " + String(keyword));
      if (doc.containsKey(keyword)) {
        ShowDebug("received position for cover #" + String(i + 1) + " :");
        ShowDebug(doc[keyword]);
        SetCoverPosition(i, doc[keyword]);
      }
    }
    doc.clear();
  }
  else if (strPayload[0] == 'R') {

    // Relais commando
    ShowDebug("Relay:");

    RelayPort = strPayload[1] - 48;
    if (RelayPort > 16) RelayPort -= 3;
    RelayValue = strPayload[2] - 48;

    if (RelayValue == 40) {
      ShowDebug("Relay " + String (RelayPins[RelayPort]));
      if (digitalRead(RelayPins[RelayPort]) == LOW) {
        digitalWrite(RelayPins[RelayPort], HIGH);
        ShowDebug("...to HIGH");
      }
      else {
        digitalWrite(RelayPins[RelayPort], LOW);
        ShowDebug("...to LOW");
      }
    } else {
      digitalWrite(RelayPins[RelayPort], RelayValue);
    }
    report_state_relay();
  } 
  else if (strPayload == "IP")  {

    // 'Show IP' commando
    mqttClient.publish(topic_out, ip.c_str());// publish IP nr
  }
  else if (strPayload == "AON") {

    // Alle relais aan
    for (byte i = 0 ; i < NumberOfRelays; i++) {
      if (RelayInitialState[i] == LOW) {
        digitalWrite(RelayPins[i], HIGH);
      } else {
        digitalWrite(RelayPins[i], LOW);
      }
    }
    report_state_relay();
  }
  else if (strPayload == "AOF") {
    // Alle relais uit
    for (byte i = 0 ; i < NumberOfRelays; i++) {
      digitalWrite(RelayPins[i], RelayInitialState[i]);
    }
    report_state_relay();
  }
  else if (strPayload == "STAT") {

    // Status van alle sensors and relais
    sendData();
  }
  else if (strPayload == "#RESET") {
    ShowDebug("Reset command received, resetting in one second...");
    delay(1000);
    resetFunc();
  }
  else if (strPayload[0] == 'P') {  // PULSE RELAY
    int PulseRelayPort = strPayload[1] - 48;
    if (PulseRelayPort < NumberOfPulseRelays) {
      if (strPayload[2] == '1') {  // Pulserelay on
        ShowDebug("Enabling pulse relay " + String(PulseRelayPort) + ".");
        digitalWrite(PulseRelayPins[PulseRelayPort], !PulseRelayInitialStates[PulseRelayPort]);
        String messageString = "P" + String(PulseRelayPort) + "1";
        messageString.toCharArray(messageBuffer, messageString.length() + 1);
        mqttClient.publish(topic_out_pulse, messageBuffer);
        PulseActivityTimes[PulseRelayPort] = millis();
        ShowDebug("Starting pulse time on " + String(millis()));
      }
      else { // Pulserelay forced off
        ShowDebug("Disabling pulse relay " + String(PulseRelayPort) + ".");
        String messageString = "P" + String(PulseRelayPort) + "0";
        messageString.toCharArray(messageBuffer, messageString.length() + 1);
        mqttClient.publish(topic_out_pulse, messageBuffer);
        digitalWrite(PulseRelayPins[PulseRelayPort], PulseRelayInitialStates[PulseRelayPort]);
      }
    }
    else {
      ShowDebug("No such pulserelay defined!");
    }
  }
  // *** Control for screen covers ****
  // Command: O + cover number [+ pulse duration in ms]
  //  opens cover:
  //    - sets direction relay: open = inverse state
  //    - pulse duration taken from command, if not given, use parameter COVERDELAYTIME
  //    - opens motor relay for duration of pulse

  else if (strPayload[0] == 'O') {
    // Cover commando: open
    ShowDebug("Cover command : Open");
    byte CoverPort = strPayload[1] - 48;
    // 2nd char is # of cover
    ShowDebug("Cover number " + String(CoverPort));
    if (CoverPort <= NumberOfCovers) {
      SetCoverPosition(CoverPort - 1, 100);
    }
    else {
      ShowDebug("No such cover defined: " + String(CoverPort));
    }
  } // End of Cover commando: open
  //
  // Command: C + cover number [+ pulse duration in ms]
  //  closes cover:
  //    - sets direction relay: open = normal state
  //    - pulse duration taken from command, if not given, use parameter COVERDELAYTIME
  //    - opens motor relay for duration of pulse
  //
  else if (strPayload[0] == 'C') {
    // Cover commando: close
    ShowDebug("Cover command : Close");
    byte CoverPort = strPayload[1] - 48;
    ShowDebug("Cover number " + String(CoverPort));
    if (CoverPort <= NumberOfCovers) {
      SetCoverPosition(CoverPort - 1, 0);
    }
    else {
      ShowDebug("No such cover defined: " + String(CoverPort));
    }
  }
  //
  // Command: S + cover number
  //  stops cover movement:
  //    - closes motor relay, stops pulse
  //
  else if (strPayload[0] == 'S') {
    // Cover commando: stop
    ShowDebug("Cover command : Stop");
    byte CoverPort = strPayload[1] - 48;
    ShowDebug("Cover number " + String(CoverPort));
    if (CoverPort <= NumberOfCovers) {
      StopCover(CoverPort - 1);
    }
    else {
      ShowDebug("No such cover defined: " + String(CoverPort));
    }
  }
  // MQTT Discovery Lock commands
  else if (strPayload[0] == 'L') {
    // Lock commando: lock
    ShowDebug("Lock command : lock");
    byte LockPort = strPayload[1] - 48;
    ShowDebug("Lock number " + String(LockPort));
    int PulseRelayPort = LockPulse[LockPort - 1];
    if (LockPort <= NumberOfLocks) {
      // End opening pulse
      digitalWrite(PulseRelayPins[PulseRelayPort], PulseRelayInitialStates[PulseRelayPort]);
      String messageString = "P" + String(PulseRelayPort) + "0";
      messageString.toCharArray(messageBuffer, messageString.length() + 1);
      mqttClient.publish(topic_out_pulse, messageBuffer);
      LockState[LockPort - 1] = 1;
      report_state_lock();
    }
    else {
      ShowDebug("No such lock defined: " + String(LockPort));
    }
  }
  else if (strPayload[0] == 'U') {
    // Lock commando: unlock
    ShowDebug("Lock command : unlock");
    byte LockPort = strPayload[1] - 48;
    ShowDebug("Lock number " + String(LockPort));
    int PulseRelayPort = LockPulse[LockPort - 1];
    if (LockPort <= NumberOfLocks) {
      // Set opening pulse
      digitalWrite(PulseRelayPins[PulseRelayPort], !PulseRelayInitialStates[PulseRelayPort]);
      String messageString = "P" + String(PulseRelayPort) + "1";
      messageString.toCharArray(messageBuffer, messageString.length() + 1);
      mqttClient.publish(topic_out_pulse, messageBuffer);
      PulseActivityTimes[PulseRelayPort] = millis();
      LockState[LockPort - 1] = 0;
      report_state_lock();
    }
    else {
      ShowDebug("No such lock defined: " + String(LockPort));
    }
  }
  else {
    // Onbekend commando
    ShowDebug("Unknown value");
    mqttClient.publish(topic_out, "Unknown command");
  }
}
void setDeviceInfo(char* configtopic) {
  JsonObject device = doc.createNestedObject("dev");
  JsonArray identifiers = device.createNestedArray("ids");
  identifiers.add(CLIENT_ID);
  JsonArray connections = device.createNestedArray("cns");
  connections.add(serialized("[\"ip\",\"" + String(ip) + "\"]"));
  connections.add(serialized("[\"mac\",\"" + mac2String(mac) + "\"]"));
  device["name"] = DISCOVERY_ID;
  device["mdl"] = MODEL_ID;
  device["mf"] = MANUFACTURER_ID;
  size_t n = serializeJson(doc, messageBuffer);
  ShowDebug("Sending MQTT config on topic:");
  ShowDebug(configtopic);
  ShowDebug("Config json:");
  ShowDebug(messageBuffer);
  if (mqttClient.publish(configtopic, messageBuffer, n)) {
    ShowDebug("...MQTT config sent.");
  }
  else {
    ShowDebug("...publish failed, either connection lost, or message too large.");
  }
}
//void SendTrigger(int Button) {
//  doc.clear();
//  doc["automation_type"] = "trigger";
//  doc["topic"] = state_topic_buttons;
//  doc["type"] = "button_short_press";
//  doc["subtype"] = "button_1";
//  setDeviceInfo((config_topic_base + "/device_automation/" + ButtonNames[Button] + "/config").c_str());
//}
void reportMQTTdisco() {
  // discovery data for relays
  for (int i = 0; i < NumberOfRelays ; i++) {
    doc.clear();
    doc["name"] = SwitchNames[i];
    doc["uniq_id"] = item_prefix + "_switch" + String(i + 1);
    doc["stat_t"] = state_topic_relays;
    doc["cmd_t"] = topic_in;
    if (!RelayInitialState[i]) {
      doc["pl_on"] = "R" + String(i) + "1";
      doc["pl_off"] = "R" + String(i) + "0";
    }
    else {
      doc["pl_on"] = "R" + String(i) + "0";
      doc["pl_off"] = "R" + String(i) + "1";
    }
    doc["stat_on"] = "on";
    doc["stat_off"] = "off";
    doc["val_tpl"] = "{{value_json.POWER" + String(i) + "}}";
    setDeviceInfo((config_topic_base + "/switch/" + item_prefix + "_switch" + String(i + 1) + "/config").c_str());
  }
  // discovery data for lights
  for (int i = 0; i < NumberOfLights ; i++) {
    doc.clear();
    doc["name"] = LightNames[i];
    doc["uniq_id"] = item_prefix + "_light" + String(i + 1);
    doc["stat_t"] = state_topic_lights + String(i + 1);
    doc["cmd_t"] = cmd_topic_lights + String(i + 1);
    doc["schema"] = "json";
    doc["brightness"] = LightBrightness[i];
    mqttClient.subscribe((cmd_topic_lights + String(i + 1)).c_str());
    setDeviceInfo((config_topic_base + "/light/" + item_prefix + "_light" + String(i + 1) + "/config").c_str());
  }
  // discovery data for covers
  for (int i = 0; i < NumberOfCovers ; i++ ) {
    doc.clear();
    doc["name"] = CoverNames[i];
    doc["uniq_id"] = item_prefix + "_cover" + String(i + 1);
    doc["pos_t"] = state_topic_covers;
    doc["cmd_t"] = topic_in;
    doc["pl_open"] = "O" + String(i + 1);
    doc["pl_cls"] = "C" + String(i + 1);
    doc["pl_stop"] = "S" + String(i + 1);
    doc["position_open"] = 100;
    doc["position_closed"] = 0;
    doc["set_pos_t"] = topic_in;
    doc["set_pos_tpl"] = "{ \"POSITION" + String(i + 1) +  "\": {{ position }} }";
    doc["val_tpl"] = "{{value_json.POSITION" + String(i + 1) + "}}";
    setDeviceInfo((config_topic_base + "/cover/" + item_prefix + "_cover" + String(i + 1) + "/config").c_str());
  }
  // discovery data for locks
  for (int i = 0; i < NumberOfLocks ; i++ ) {
    doc.clear();
    doc["name"] = LockNames[i];
    doc["uniq_id"] = item_prefix + "_lock" + String(i + 1);
    doc["stat_t"] = state_topic_locks;
    doc["cmd_t"] = topic_in;
    doc["pl_lock"] = "L" + String(i + 1);
    doc["pl_unlk"] = "U" + String(i + 1);
    doc["state_locked"] = "LOCKED";
    doc["state_unlocked"] = "UNLOCKED";
    doc["val_tpl"] = "{{value_json.LOCK" + String(i + 1) + "}}";
    setDeviceInfo((config_topic_base + "/lock/" + item_prefix + "_lock" + String(i + 1) + "/config").c_str());
  }
  // discover data for pirs (binary_sensors)
  for (int i = 0; i < NumberOfPirs ; i++ ) {
    doc.clear();
    doc["name"] = PirNames[i];
    doc["uniq_id"] = item_prefix + "_pir" + String(i + 1);
    doc["stat_t"] = state_topic_pirs;
    doc["device_class"] = PirClasses[i];
    doc["pl_on"] = "ON";
    doc["pl_off"] = "OFF";
    doc["val_tpl"] = " {{value_json.PIR" + String(i + 1) + "}}";
    setDeviceInfo((config_topic_base + "/binary_sensor/" + item_prefix + "_pir" + String(i + 1) + "/config").c_str());
  }
  // discover data for buttons (triggers)
  //  for (int i = 0; i < NumberOfButtons ; i++ ) {
  //    doc.clear();
  //    doc["automation_type"] = "trigger";
  //    doc["topic"] = state_topic_buttons;
  //    doc["type"] = "button_short_press";
  //    doc["subtype"] = "button_1";
  //    setDeviceInfo((config_topic_base + "/device_automation/" + ButtonNames[i] + "/config").c_str());
  //  }
  // end send config data for MQTT discovery
  // discover data for sensors (sensors)
  for (int i = 0; i < NumberOfSensors ; i++ ) {
    doc.clear();
    doc["name"] = SensorNames[i];
    doc["uniq_id"] = item_prefix + "_sensor" + String(i + 1);
    doc["stat_t"] = state_topic_sensors;
    if (SensorClasses[i] != "") {
      doc["device_class"] = SensorClasses[i];
    }
    doc["unit_of_meas"] = SensorUnits[i];
    doc["val_tpl"] = " {{value_json.sensor" + String(i + 1) + " | round (1) }}";
    setDeviceInfo((config_topic_base + "/sensor/" + item_prefix + "_sensor"  + String(i + 1) + "/config").c_str());
  }
  //  end send config data for MQTT discovery
}
void setup() {

  if (debug) {
    Serial.begin(115200);
    ShowDebug(CLIENT_ID);
    ShowDebug(String(MQTT_MAX_PACKET_SIZE));
  }

#if defined(MCP_present)
  Wire.setClock(400000);
  mcp.begin();      // use default address 0 for i2c expander (MCP23017/MCP23008)
#endif

  for (byte thisPin = 0; thisPin < NumberOfRelays; thisPin++) {
    ShowDebug("Relay: " + String(RelayPins[thisPin]));
    if (RelayPins[thisPin] < 100) { // pins > 100 are MCP ports
      pinMode(RelayPins[thisPin], OUTPUT);
      digitalWrite(RelayPins[thisPin], RelayInitialState[thisPin]);
    }
#if defined(MCP_present)
    else {
      mcp.pinMode(RelayPins[thisPin] - 100, OUTPUT);
      mcp.digitalWrite(RelayPins[thisPin] - 100, RelayInitialState[thisPin]);
    }
#endif
  }

  for (byte thisPin = 0; thisPin < NumberOfLights; thisPin++) {
    ShowDebug("Light: " + String(LightPins[thisPin]));
    if (LightPins[thisPin] < 100) { // pins > 100 are MCP ports
      pinMode(LightPins[thisPin], OUTPUT);
      digitalWrite(LightPins[thisPin], LightInitialState[thisPin]);
    }
#if defined(MCP_present)    
    else {
      mcp.pinMode(LightPins[thisPin] - 100, OUTPUT);
      mcp.digitalWrite(LightPins[thisPin] - 100, LightInitialState[thisPin]);
    }
#endif
  }

  for (int thisPin = 0; thisPin < NumberOfPulseRelays; thisPin++) {
    ShowDebug("Pulse relay: " + String(PulseRelayPins[thisPin]));
    if (PulseRelayPins[thisPin] < 100) { // pins > 100 are MCP ports    
      pinMode(PulseRelayPins[thisPin], OUTPUT);
      digitalWrite(PulseRelayPins[thisPin], PulseRelayInitialStates[thisPin]);
    }
#if defined(MCP_present)  
    else {
      mcp.pinMode(PulseRelayPins[thisPin] - 100, OUTPUT);
      mcp.digitalWrite(PulseRelayPins[thisPin] - 100, PulseRelayInitialStates[thisPin]);
    }
#endif
  }

  for (int thisButton = 0; thisButton < NumberOfButtons; thisButton++) {
    ShowDebug("Button: " + String(ButtonPins[thisButton]));
    if (ButtonPins[thisButton] < 100) { // pins > 100 are MCP ports 
      pinMode(ButtonPins[thisButton], INPUT_PULLUP);
    }
#if defined(MCP_present)
    else {
      mcp.pinMode(ButtonPins[thisButton] - 100, INPUT_PULLUP);
    } 
#endif 
  }

  for (byte pirid = 0; pirid < NumberOfPirs; pirid++) {
    ShowDebug("Pir: " + String(PirSensors[pirid]));
    lastPirStates[pirid] = PirInitialState[pirid];
    if (PirSensors[pirid] < 100) { // pins > 100 are MCP ports 
      pinMode(PirSensors[pirid], INPUT_PULLUP);
    }
#if defined(MCP_present)
    else {
      mcp.pinMode(PirSensors[pirid] - 100, INPUT_PULLUP);
    }
#endif
  }

#if defined(DHT_present)
  dht.begin();
  ShowDebug("DHT-22 sensor: 3");
#endif

#if defined(LDR_present)
  pinMode(LightSensor, INPUT);
  ShowDebug("LDR sensor: A10");
#endif

#if defined(DS18B20_present)
  ShowDebug("DS18B20 sensor: 16");
#endif

#if defined(BMP_present)
  if (!bmp.begin()) {
    ShowDebug("Could not find a valid BMP280 sensor, check wiring!");
  }
  else {
    ShowDebug("BMP280 sensor (i2c): 20 (SDA), 21 (SCL)");
  }
#endif

#if defined(MQ_present)
  pinMode(SmokeSensor, INPUT);
  ShowDebug("MQ2 sensor: " + String(SmokeSensor));
#endif

#if defined(MQ7_present)
  mq_millis = millis();
  ShowDebug("MQ7 sensor: " + String(mq_sensor_pin));
#endif

  ShowDebug("Network...");
  // attempt to connect to network:
  //   setup ethernet communication using DHCP
  if (Ethernet.begin(mac) == 0) {

    ShowDebug(F("No DHCP"));
    delay(1000);
    resetFunc();
  }
  ShowDebug(F("Ethernet via DHCP"));
  ShowDebug("IP address: ");
  ip = String (Ethernet.localIP()[0]);
  ip = ip + ".";
  ip = ip + String (Ethernet.localIP()[1]);
  ip = ip + ".";
  ip = ip + String (Ethernet.localIP()[2]);
  ip = ip + ".";
  ip = ip + String (Ethernet.localIP()[3]);
  ShowDebug(ip);
  ShowDebug("");

  // setup mqtt client
  mqttClient.setClient(ethClient);
  mqttClient.setServer(MQTTSERVER, 1883); // or local broker
  ShowDebug("MQTT set up");
  mqttClient.setCallback(callback);
  ShowDebug("Ready to send data");
  lastPublishTime = millis();
}
void loop() {
  // Main loop, where we check if we're connected to MQTT...
  if (!mqttClient.connected()) {
    ShowDebug("Not Connected!");
    reconnect();
    startsend = true;
  }

  // ... then send all relay stats and discovery info when we've just started up....
  if (startsend) {
    reportMQTTdisco();
    report_state();
    startsend = false;
  }

  // ...handle the PulseRelays, ...
  for (int id = 0; id < NumberOfPulseRelays; id++) {
    ProcessPulseRelays(id);
  }

  // handle the cover position
  for (int id = 0; id < NumberOfCovers; id++) {
    ProcessCovers(id);
  }

  // ...read out the PIR sensors...
  for (int id = 0; id < NumberOfPirs; id++) {
    check_pir(id);
  }

#if defined(MQ7_present)
  // ...process the MQ-7 sensor...
  if (mq_state == 0) {
    digitalWrite(mq_state_pin, HIGH);
    if ( mq_millis < millis() ) {
      ShowDebug("MQ-7 measure...");
      mq_state = 1;
      mq_millis = millis() + mq_measure_interval;
    }
  }
  else {
    digitalWrite(mq_state_pin, LOW);
    int mq_value = analogRead(mq_sensor_pin);
    //    ShowDebug("Value: "+String(mq_value));
    if ( (mq_millis + mq_startup - mq_measure_interval) < millis() )
    {
      co_value *= 0.999;
      co_value += 0.001 * mq_value;
    }
    if ( mq_millis < millis() ) {
      ShowDebug("MQ-7 heatup...");
      mq_state = 0;
      mq_millis = millis() + mq_heat_interval;
    }
  }
#endif

  // ...see if it's time to send new data, ....
  if (millis() - lastPublishTime > PUBLISH_DELAY) {
    lastPublishTime = millis();
    sendData();
  }
  else if (millis() - lastReportTime > REPORT_DELAY) {
    lastReportTime = millis();
    report_state();
  }
  else {
    for (int id = 0; id < NumberOfButtons; id++) {
      processButtonDigital(id);
    }
  }

  // and loop.
  mqttClient.loop();
}