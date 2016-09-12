#include <HashMap.h>

const byte HASH_SIZE = 5;
// storage
HashType<int,int> hashRawArray[HASH_SIZE];
HashMap<int, int> hashMap = HashMap<int,int>( hashRawArray, HASH_SIZE );

void setup() {
  // put your setup code here, to run once:

  hashMap[0](3,18);
  hashMap[1](6,200);
  hashMap[2](9,1234);
  hashMap[3](12,123);
  hashMap[4](15,20);

  Serial.begin(115200);
  Serial.println( hashMap.getIndexOf(7),DEC );
  Serial.println( hashMap.getValueOf("test") );

  hashMap.debug();

    for (int i=0; i < HASH_SIZE ; i++)
      {
      Serial.print(i);
      Serial.print(" : ");
      Serial.print( hashMap.getIndexOf(i), DEC );
      Serial.print(" : ");
      Serial.println(hashMap.getValueOf(i));
      }

}


void loop() {
  // put your main code here, to run repeatedly:

}
