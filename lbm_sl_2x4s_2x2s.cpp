void setup()
{
  Serial.begin(9600); //Start serial connection with computer
}
 
void loop()
{            // read analog pins 0-2 and report results by serial
    for (int adcPin = 0; adcPin < 13; adcPin++) {
        analogRead(adcPin);    // first adc reading is discarded
        delay(20);
        int reading = 0;       // now we read the pin 5 times
        for (int loop = 0; loop < 5; loop++)
        {
           reading += analogRead(adcPin);  // add each value
           delay(20);
        }       
        // now divide by 5 for average and convert to a voltage
        float voltage = reading * 5.0 / 5.0; 
        voltage /= 1023.0;
        float adc = reading / 5.0;
             // send output to Pi through serial port
        if (adcPin != 12) {
          Serial.print(voltage);
          Serial.print(',');
        } else {
          Serial.println(voltage);
        }
    }  
    delay(3000);   // wait a second then read them all again
} 
