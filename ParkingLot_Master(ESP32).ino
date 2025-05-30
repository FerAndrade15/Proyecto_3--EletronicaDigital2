// Libraries
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>

// I2C Addresses
#define DataServer1 0x20
#define DataServer2 0x30
#define I2C_SDA 21
#define I2C_SCL 22

// WiFi Credentials
const char* ssid = "Al";
const char* password = "hola123u";

WebServer server(80);

// Variables para lectura I2C
uint8_t halfkeys1 = 0;
uint8_t halfkeys2 = 0;
uint8_t keys = 0;

// Nombres de espacios de parqueo
const char* slotNames[8] = {"A1","A2","A3","A4","A5","A6","A7","A8"};

void setup() {
  Serial.begin(115200);
  Serial.println("Try connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED){
    delay(1000);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected successfully");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Rutas para página y datos
  server.on("/", handle_OnConnect);
  server.on("/LastReading", handle_CarsChange);

  server.begin();
  Serial.println("HTTP server started");

  // Inicializar I2C
  Wire.begin(I2C_SDA, I2C_SCL);
}

void loop() {
  server.handleClient();

  //Lectura de los registros de la núcleo
  if(Wire.requestFrom(DataServer1, 1))   halfkeys1 = Wire.read();
  if(Wire.requestFrom(DataServer2, 1))   halfkeys2 = Wire.read();

  // Muestra de los datos recibido como método de control
  Serial.printf("STM 1: %u\n", halfkeys1);
  Serial.printf("STM 2: %u\n", halfkeys2);

  //Reorganización de datos
  keys = (halfkeys1 << 4) | halfkeys2;

  //Envío de unión de valores para actualizar los datos
  Wire.beginTransmission(DataServer1);
  Wire.write(keys);
  Wire.endTransmission(true);
  Wire.beginTransmission(DataServer2);
  Wire.write(keys);
  Wire.endTransmission(true);

  Serial.printf("Return to STM's: %u\n", keys);

  delay(100);
}

// Handle para iniciar la conexión
void handle_OnConnect() {
  keys = 0;  //Inicio de la lectura, con los sensores completamente reseteados         
  Serial.println("Starting server with all empty");
  server.send(200, "text/html", SendHTML(0, keys));
}

// Handle para reenviar la información recopilada entre ambas STM
void handle_CarsChange() {
  uint8_t count = 0;
  for(int i=0; i<8; i++){
    if(keys & (1 << i)) count++;
  }
  Serial.printf("Cantidad de autos detectados: %u\n", count);
  server.send(200, "text/html", SendHTML(count, keys));
}

// Generación del código para la página HTML
String SendHTML(uint8_t activated, uint8_t position){
  String html_code = "";
  html_code += "<!DOCTYPE html>\n";
  html_code += "<html lang=\"es\">\n";
  html_code += "<head>\n";
  html_code += "<title>Parqueomatic - Control de parqueos</title>\n";
  html_code += "<meta charset=\"utf-8\">\n";
  html_code += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n";
  html_code += "<meta http-equiv=\"refresh\" content=\"1\">\n";
  html_code += "<link rel=\"stylesheet\" href=\"https://cdn.jsdelivr.net/npm/bootstrap@4.6.2/dist/css/bootstrap.min.css\">\n";
  html_code += "<script src=\"https://cdn.jsdelivr.net/npm/jquery@3.7.1/dist/jquery.slim.min.js\"></script>\n";
  html_code += "<script src=\"https://cdn.jsdelivr.net/npm/popper.js@1.16.1/dist/umd/popper.min.js\"></script>\n";
  html_code += "<script src=\"https://cdn.jsdelivr.net/npm/bootstrap@4.6.2/dist/js/bootstrap.bundle.min.js\"></script>\n";
  html_code += "</head>\n";
  html_code += "<body>\n";
  html_code += "<div class=\"container-xxl bg-dark text-white\">\n";
  html_code += "<figure class=\"text-center\">\n";
  html_code += "<br>\n";
  html_code += "<h1>Control de parqueos</h1>\n";
  html_code += "<br>\n";
  html_code += "</figure>\n";
  html_code += "</div>\n";

  html_code += "<style>\n";
  html_code += ".custom-bg {\n";
  html_code += "background-image: url('empty.png');\n";
  html_code += "background-size: contain;\n";
  html_code += "background-position: center;\n";
  html_code += "background-repeat: no-repeat;\n";
  html_code += "min-height: 650px;\n";
  html_code += "color: #6C757D;\n";
  html_code += "padding: 20px;\n";
  html_code += "position: relative;\n";
  html_code += "}\n";
  html_code += ".car {\n";
  html_code += "position: absolute;\n";
  html_code += "width: 100px;\n";
  html_code += "height: auto;\n";
  html_code += "transform: translate(-51%, -50%);\n";
  html_code += "z-index: 10;\n";
  html_code += "}\n";
  html_code += "</style>\n";

  html_code += "<div class=\"container-xxl bg-secondary text-white\">\n";
  html_code += "<div class=\"row justify-content-center align-items-center\" style=\"min-height: 650px;\">\n";
  html_code += "<div class=\"col-6 custom-bg\">\n";

  const char* posStyles[8] = {
    "top: 25%; left:24%;",
    "top: 25%; left:42%;",
    "top: 25%; left:59%;",
    "top: 25%; left:76%;",
    "top: 75%; left:24%;",
    "top: 75%; left:42%;",
    "top: 75%; left:59%;",
    "top: 75%; left:76%;"
  };

  for (int i = 0; i < 8; i++) {
    bool occupied = position & (1 << i);
    // Imagen arriba para A1-A4, abajo para A5-A8
    const char* img_src = (i < 4) ? "car_up.png" : "car_down.png";
    html_code += "<img src=\"";
    html_code += img_src;
    html_code += "\" id=\"";
    html_code += slotNames[i];
    html_code += "\" alt=\"Carro\" class=\"car\" style=\"";
    html_code += posStyles[i];
    html_code += "\" ";
    html_code += occupied ? "visible" : "hidden"; // Mostrar solo si ocupado
    html_code += " />\n";
  }

  html_code += "</div>\n";

  html_code += "<div class=\"col-6 d-flex flex-column align-items-center justify-content-center\">\n";
  html_code += "<div class=\"mb-3\">\n";
  html_code += "<div class=\"dropdown\">\n";
  html_code += "<button class=\"btn btn-dark dropdown-toggle\" type=\"button\" id=\"basementDropdown\" data-toggle=\"dropdown\" aria-haspopup=\"true\" aria-expanded=\"false\">Basement Level</button>\n";
  html_code += "<div class=\"dropdown-menu\" aria-labelledby=\"basementDropdown\">\n";
  html_code += "<a class=\"dropdown-item\" href=\"#\">Basement 1</a>\n";
  html_code += "</div>\n";
  html_code += "</div>\n";
  html_code += "</div>\n";

  html_code += "<div id=\"selectedLevel\" class=\"mb-3 text-light font-weight-bold\">\n";
  html_code += "Selected Level: Level 1\n";
  html_code += "</div>\n";

  html_code += "<div class=\"table-responsive\" style=\"max-width: 400px; max-height: 500px; overflow-y: auto;\">\n";
  html_code += "<table class=\"table table-dark table-striped table-hover table-bordered\">\n";
  html_code += "<thead class=\"table-light text-dark\">\n";
  html_code += "<tr><th>Space</th><th>Status</th></tr>\n";
  html_code += "</thead>\n";
  html_code += "<tbody>\n";

  for (int i=0; i<8; i++){
    html_code += "<tr><td>";
    html_code += slotNames[i];
    html_code += "</td><td><span class=\"badge ";
    if(position & (1 << i)){
      html_code += "bg-danger\">Ocuppied";
    } else {
      html_code += "bg-success\">Available";
    }
    html_code += "</span></td></tr>\n";
  }

  html_code += "</tbody>\n";
  html_code += "</table>\n";
  html_code += "</div>\n";

  html_code += "</div>\n"; 
  html_code += "</div>\n"; 
  html_code += "</div>\n"; 
  html_code += "</body>\n";
  html_code += "</html>\n";

  return html_code;
}
