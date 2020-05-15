
#include "dsmr.h"

/**
   Define the data we're interested in, as well as the datastructure to
   hold the parsed data. This list shows all supported fields, remove
   any fields you are not using from the below list to make the parsing
   and printing code smaller.
   Each template argument below results in a field of the same name.
*/
using MyData = ParsedData <
               /* String */ identification,
               /* String */ p1_version,
               /* String */ timestamp,
               /* String */ equipment_id,
               /* FixedValue */ energy_delivered_tariff1,
               /* FixedValue */ energy_delivered_tariff2,
               /* FixedValue */ energy_returned_tariff1,
               /* FixedValue */ energy_returned_tariff2,
               /* String */ electricity_tariff,
               /* FixedValue */ power_delivered,
               /* FixedValue */ power_returned,
               /* uint32_t */ electricity_failures,
               /* uint32_t */ electricity_long_failures,
               /* uint32_t */ electricity_sags_l1,
               /* uint32_t */ electricity_swells_l1,
               /* String */ message_long,
               /* FixedValue */ voltage_l1,
               /* FixedValue */ current_l1,
               /* FixedValue */ power_delivered_l1,
               /* FixedValue */ power_returned_l1,
               /* uint16_t */ gas_device_type,
               /* String */ gas_equipment_id,
               /* TimestampedFixedValue */ gas_delivered
               >;

struct Printer {
  template<typename Item>
  void apply(Item &i) {
    if (i.present()) {
      Serial.print(Item::name);
      Serial.print(F(": "));
      Serial.print(i.val());
      Serial.print(Item::unit());
      Serial.println();
    }
  }
};

P1Reader reader(&Serial1, 2);

const int READER_INTERVAL = 5000; // interval to read meter values in ms
char c;
unsigned long last_p1_read;
MyData last_p1_data;

void types(String a){Serial.println("it's a String");}
void types(int a)   {Serial.println("it's an int");}
void types(char* a) {Serial.println("it's a char*");}
void types(float a) {Serial.println("it's a float");} 

void PrintValues(MyData data) {
  Serial.print("Id: ");
  Serial.println(data.identification);
  Serial.print("Versie: ");
  Serial.println(data.p1_version);
  Serial.print("Tijd: ");
  Serial.println(data.timestamp);
  Serial.print("Equipment ID: ");
  Serial.println(data.equipment_id);
  Serial.print("Tarief (1=nacht, 2=dag): ");
  Serial.println(data.electricity_tariff);
  types(data.electricity_tariff);
  Serial.print("Verbruik: ");
  Serial.print(data.power_delivered.int_val());
  Serial.println(" W.");
  Serial.print("Voltage: ");
  Serial.print(data.voltage_l1.val(), 0);
  Serial.println(" V.");
  Serial.print("Stroomsterkte: ");
  Serial.print(data.current_l1);
  Serial.println(" A.");
  Serial.print("Verbruik nacht: ");
  Serial.print(data.energy_delivered_tariff1.val(), 0);
  Serial.println(" kWh.");
  Serial.print("Verbruik dag: ");
  Serial.print(data.energy_delivered_tariff2.val(), 0);
  Serial.println(" kWh.");
  Serial.print("Elektra Storingen: ");
  Serial.println(data.electricity_failures);
  Serial.print("Elektra Lange Storingen: ");
  Serial.println(data.electricity_long_failures);
  Serial.print("Elektra Brownouts: ");
  Serial.println(data.electricity_sags_l1);
  Serial.print("Elektra Pieken: ");
  Serial.println(data.electricity_swells_l1);
  Serial.print("Gas type: ");
  Serial.println(data.gas_device_type);
  Serial.print("Gas id: ");
  Serial.println(data.gas_equipment_id);
  Serial.print("Gasverbruik: ");
  Serial.print(data.gas_delivered.val(), 0);
  Serial.println(" m3.");
  Serial.println("==============================");
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial.println("Setup done");
  reader.enable(true);
  last_p1_read = millis();
}

void loop() {
  // Allow the reader to check the serial buffer regularly
  
  reader.loop();

  // Every minute, fire off a one-off reading
  unsigned long now = millis();
  if (now - last_p1_read > READER_INTERVAL) {
    reader.enable(true);
    last_p1_read = now;
  }
  if (reader.available()) {
    MyData data;
    String err;
    if (reader.parse(&data, &err)) {
      // Parse succesful, print result
      //      data.applyEach(Printer());
      last_p1_data = data;
      PrintValues(data);
    } else {
      // Parser error, print error
      Serial.println(err);
    }
  }
}
