
## HTTP client code

Don't forget to install ArduinoHTTPClient library in Sketch
```
#include <ArduinoHttpClient.h>
```


Global variables
```
// Create these as global variables at the top of your file with other globals
WiFiClient wifi;
HttpClient httpClient = HttpClient(wifi, "fn-obs-backend01.azurewebsites.net", 80);  // Replace with your server address and port

```

In loop

```
loop()

  ...

  Serial.println("Making HTTP request...");
  httpClient.get("/api/backendfunction1?name=JV&code=<secret>");
  
  // Get the status code and response
  int statusCode = httpClient.responseStatusCode();
  String response = httpClient.responseBody();
  
  Serial.print("Status code: ");
  Serial.println(statusCode);
  Serial.print("Response: ");
  Serial.println(response);

```

## Theme music

```
  // BUZZ - Replace with new musical sequence
  // Define note frequencies
  const int A3 = 220;    // A3
  const int AS3 = 233;   // A#3
  const int B3 = 247;    // B3
  const int CS4 = 277;   // C#4
  const int DS4 = 311;   // D#4
  const int E4 = 330;    // E4
  const int FS3 = 185;   // F#3
  
  // Define note durations (in milliseconds)
  const int QUARTER = 250;
  const int HALF = 500;
  const int WHOLE = 1000;
  const int TRIPLET = 200;  // QUARTER * 2/3
  
  // Play the sequence
  // F#3 (HALF)
  tone(A2, FS3, HALF);
  delay(HALF + 50);
  
  // E4 (HALF + QUARTER)
  tone(A2, E4, HALF + QUARTER);
  delay(HALF + QUARTER + 50);
  
  // D#4 (QUARTER)
  tone(A2, DS4, QUARTER);
  delay(QUARTER + 50);
  
  // Triplet sequence: C#4, B3, A#3
  tone(A2, CS4, TRIPLET);
  delay(TRIPLET + 20);
  tone(A2, B3, TRIPLET);
  delay(TRIPLET + 20);
  tone(A2, AS3, TRIPLET);
  delay(TRIPLET + 20);
  
  // A3 (WHOLE)
  tone(A2, A3, WHOLE);
  delay(WHOLE + 50);
  
  noTone(A2);  
```