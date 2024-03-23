#include <usb_serial.h>

bool ledState = true;
void setup(){
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN,ledState);
    Serial.begin(9600);
    while (!Serial){}
}

String get_pentadecimal(int number){
    return String(number, 15);
}
String get_pentadecimal(float number, int precision = 0){
    int wholePart = (int)number;
    float decimalNumber = (number - wholePart);
    int decimalPlaces = get_decimal_places(number);
    if (decimalPlaces > precision){
        decimalPlaces = precision;
    }
    Serial.println((String)"Decimal places: "+String(decimalPlaces));
    int decimalPart = decimalNumber * pow(10, decimalPlaces);

    Serial.println((String)"Decimal part: "+String(decimalPart));
    String output = "";
    output += get_pentadecimal(wholePart);
    if (precision > 0){
        output += "ff";
        for (int i = 0; i < get_leading_zeros(decimalPart, decimalPlaces); i++){
            output += "0";
        }
        output += get_pentadecimal(decimalPart);
    }
    return output;

}

int get_decimal_places(float number){
    int counter = 0;
    while (number != (int)number){
        number *= 10;
        counter++;
    }
    return counter;
}

int get_leading_zeros(int number, int decimalPlaces){
    int target = pow(10, decimalPlaces-1);
    int counter = 0;
    while (target-number>0){
        number *= 10;
        counter++;
    }
    return counter;
}

int counter = 0;
void loop(){
    float temp = tempmonGetTemp();
    Serial.print(temp,1);
    Serial.print("*");
    Serial.println(counter);
    Serial.print(get_pentadecimal(temp,1));
    Serial.print("*");
    Serial.println(get_pentadecimal(counter++));
    delay(500);
    ledState = !ledState;
    digitalWrite(LED_BUILTIN,ledState);
}