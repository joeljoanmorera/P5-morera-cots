# Práctica 5. Gerard Cots y Joel J. Morera

## Ejercicio practico 1 : Escáner I2C

###### **Funcionamiento**

p

###### **Código del programa**

```cpp
#include <Arduino.h>

#include <Wire.h>


void setup()
{
  Wire.begin();
 
  Serial.begin(115200);
  while (!Serial);             // Leonardo: wait for serial monitor
  Serial.println("\nI2C Scanner");
}
 
 
void loop()
{
  byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
 
  delay(5000);           // wait 5 seconds for next scan
}
```

###### **Salida del puerto serie**

p

```

```

## Ejercicio practico 2 : Utilización de dispositivo I2C

###### **Funcionamiento**

p

###### **Código del programa**

p

```cpp

```

###### **Salida del puerto serie**

p

```

```

###### **Montaje**

![Montage dispositivo I2C](./images/i2c.png)

## Ejercicio de subida de nota. Parte 1 : Dispositivo que muestra frecuencia cardiaca y el contenido de oxigeno

###### **Funcionamiento**

p

###### **Código del programa**

p

```cpp

```

###### **Salida del display**

![Display](./images/display.png)

###### **Montaje**

![Montage display](./images/i2c_p1.png)

## Ejercicio de subida de nota. Parte 2 : Página web donde consultar frecuencia cardiaca y el contenido de oxigeno

###### **Funcionamiento**

p

###### **Código del programa**

p

```cpp

```

###### **Visualización de la página web**

p

```

```

###### **Montaje**

![Montage pagina web](./images/i2c_p2.png)