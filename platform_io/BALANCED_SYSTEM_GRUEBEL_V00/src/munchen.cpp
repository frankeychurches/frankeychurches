///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
// Programm zu Lageregelung der Testplattform mit Hilfe von Reaktionskreiseln.
// Mit UM6-LT IMU als Trägheitsplattform, XBee-Funkmodul zum seriellen Datenübertraggung und mbed
// Mikrokontroller als Regelungs- und Stuerungscomputer.
// Im Programm sind ein PID- und ein PD-Regler implementiert.                                       
//
// Datum: 10.01.2014  Autor: Grübel Dimitri
// 
// Programa para el control de la posición de la plataforma de pruebas mediante giroscopios de reacción.
// Con UM6-LT IMU como plataforma inercial, módulo de radio XBee para la transferencia de datos en serie y mbed
// Microcontrolador como ordenador de control y actuación.
// En el programa se implementan un controlador PID y un controlador PD.                                       
// Fecha: 10.01.2014 Autor: Grübel Dimitri
///////////////////////////////////////////////////////////////////////////////////////////////////
 
#include <mbed.h>
#include <MODSERIAL.h>                           // MBED BUFFERED SERIAL HEADER
#include <UM6_usart.h>                           // UM6 USART HEADER
#include <UM6_config.h>                          // UM6 CONFIG HEADER
 
                                                 // KOMMUNIKATIONSART MIT MBED: USB/XBEE
Serial ios(p28, p27);                            // Serielle Verbi. mit XBee über Pin: tx-28, rx-27
//Serial ios(USBTX, USBRX);                      // Serielle Verbi. über USB Port vom PC
 
DigitalOut rst(p11);                             // Digital Reset für the XBee, 200ns für reset
PwmOut x_kreisel(p21);    //kreisel = disco      // Pin21-PwmOut ist für Drehung (rotacion) um X-Achse
PwmOut y_kreisel(p23);                           // Pin23-PwmOut ist für Drehung um Y-Achse
PwmOut z_kreisel(p22);                           // Pin22-PwmOut ist für Drehung um Z-Achse
 
                                                 // EL PWM SE AJUSTA AQUI:
const float pulsweite = 10.0;                    // ancho de pulso de la señal de control in ms
const float pwwork = 2.0;                        // Anchod e pulso de la señal de trabajo in ms
const float startpw = 0.8;                       // Ancho de pulso de inicio in ms
const float pwmfakt = (pwwork - startpw)/200.0;  // Ancho de pulso para 1% de potencia
const float max_leistung = 75.0;                 // Limite de potencia global in % (leistung=rendimiento)
const float min_leistung = 10.0;                 // Potencia minima necesaria del motor in %
 
const float x_kp_min = 0.0;                      // min/max - Variablen kp für X/Y/Z-Achse
const float x_kp_max = 3.0;
const float y_kp_min = 0.0;
const float y_kp_max = 3.0;
const float z_kp_min = 0.0;
const float z_kp_max = 3.0;
 
const float x_kd_min = 0.0;                      // min/max - Variablen kd für X/Y/Z-Achse
const float x_kd_max = 3.0;
const float y_kd_min = 0.0;
const float y_kd_max = 3.0;
const float z_kd_min = 0.0;
const float z_kd_max = 3.0;
 
const float x_ki_min = 0.0;                      // min/max - Variablen ki für X/Y/Z-Achse
const float x_ki_max = 0.5;
const float y_ki_min = 0.0;
const float y_ki_max = 0.5;
const float z_ki_min = 0.0;
const float z_ki_max = 0.5;
 
const float x_winkel_min = -45.0;                // Winkelbegrenzung für X/Y/Z-Achse
const float x_winkel_max =  45.0;                // Limitacion del angulo para ejes xyz
const float y_winkel_min = -45.0;
const float y_winkel_max =  45.0;
const float z_winkel_min = -90.0;
const float z_winkel_max =  90.0;
 
const float regelgenauigkeit = 1.5;              // Regelgenauigkeit in Grad
                                                 // Precisión del control en grados
                                                 
const float x_kp_def = 1.0;                      // DEFAULT REGELPARAMETER FÜR X/Y/Z-ACHSE
const float x_kd_def = 0.4;                      // parámetros de control por defecto para ejes xyz
const float x_ki_def = 0.01;
const float y_kp_def = 1.0; 
const float y_kd_def = 0.4;
const float y_ki_def = 0.01;
const float z_kp_def = 1.0; 
const float z_kd_def = 0.4;
const float z_ki_def = 0.01;
 
DigitalOut uart_activity(LED1);                  // LED1 = UM6 SERIAL für Kommunikation
       
                                                                   
/// FUNKTIONEN ////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
 
 
///////////////////////////////////////////////////////////////////////////////////////////////////
// rxCallback // FUNKTION FÜR INTERRUPT - ABFRAGE DER DATEN DIE UM6-LT SENDET
                                          //consultar datos que envia UM6LT
 
void rxCallback(MODSERIAL_IRQ_INFO *q) 
{
    if (um6_uart.rxBufferGetCount() >=  MAX_PACKET_DATA) 
    {
        uart_activity = !uart_activity;          // LED leuchtet wenn RxBuff hat > 40 Bytes
        Process_um6_packet();                   // LED se ilumina cuando RxBuff tiene ...
    }
}
 
 
///////////////////////////////////////////////////////////////////////////////////////////////////
// calcStellwert // FUNKTION ZUR STELLWERTBERECHNUNG FÜR MBED-PWM
                    //funcion para el calculo del valor de actuacion para una PWM determinada
 
float calcStellwert(float leistung)  //rendimiento, potencia
{                                
    return (startpw + ((leistung + 100) * pwmfakt)) / pulsweite; 
}
 
 
///////////////////////////////////////////////////////////////////////////////////////////////////
// print_gyro_data // FUNKTION ZUR AUSGABE DER AKTUELLEN WINKEL UND WINKELGESCHWINDIGKEITEN
                    // FUNCIÓN PARA LA SALIDA DE LOS ÁNGULOS Y VELOCIDADES ANGULARES ACTUALES
void print_gyro_data()
{                                              
    ios.printf("Gyro_Proc_X     %+6.1f deg/s\n", data.Gyro_Proc_X);
    ios.printf("Gyro_Proc_Y     %+6.1f deg/s\n", data.Gyro_Proc_Y);
    ios.printf("Gyro_Proc_Z     %+6.1f deg/s\n", data.Gyro_Proc_Z);
    ios.printf("Roll            %+6.1f deg\n", data.Roll);
    ios.printf("Pitch           %+6.1f deg\n", data.Pitch);
    ios.printf("Yaw             %+6.1f deg\n\n", data.Yaw);
}
 
 
///////////////////////////////////////////////////////////////////////////////////////////////////
// print_gyro_angles // FUNKTION ZUR AUSGABE DER AKTUELLEN WINKEL
                     // FUNCIÓN PARA LA SALIDA DEL ÁNGULO ACTUAL
void print_gyro_angles()
{                                            
    ios.printf("Roll            %+6.1f deg\n", data.Roll);
    ios.printf("Pitch           %+6.1f deg\n", data.Pitch);
    ios.printf("Yaw             %+6.1f deg\n\n", data.Yaw);
}
 
 
///////////////////////////////////////////////////////////////////////////////////////////////////
// LageregelungAchse     // FUNKTION ZUR LAGEREGELUNG FÜR EINE ACHSE - STATISCH - PD Regler
// Posicion control eje  // FUNCIÓN DE CONTROL DE POSICIÓN PARA UN EJE - ESTÁTICO -
void LageregelungAchse(float kp, float kd, float winkel, const float& gyro_veloc, 
                       const float& gyro_angle, PwmOut& kreisel)
{                   
    float start_angle = gyro_angle;                     //Define el primer ángulo que ve como el ángulo de comienzo
    float winkel_soll = gyro_angle + winkel;                                                      
    float abweichung = gyro_angle - winkel_soll;      // desviacion = gyro_angle - angulo
    float leistung_alt = 0;                           // Variablen für Bremsverfahren -- Variables para sistema frenado
    bool start = true;
    
    while (fabs(abweichung) > regelgenauigkeit)           //desviacion > precision en el control
    {
        float leistung = abweichung * kp - gyro_veloc * kd; //potencia = desviacion * kp - gyro_veloc * kd
        if (leistung > max_leistung) leistung = max_leistung;
        if (leistung < -max_leistung) leistung = -max_leistung;
        if (leistung > -min_leistung && leistung < -0.001) leistung = -min_leistung;
        if (leistung < min_leistung && leistung > 0.001) leistung = min_leistung;
         
        int Bremsrichtung = 0;                        // Vorzeichenbestimmung der Abbremsrichtung
                                                        // Determinación del signo de la dirección de deceleración
        if (leistung >= 0)
            Bremsrichtung = -1;                      // Direccion de frenado
        else 
            Bremsrichtung = 1;
        
        float Stellwert = calcStellwert(leistung);    // Berechnung Stellwert -- Cálculo del valor de accionamiento
        kreisel.write(Stellwert);                     // Befehl an Motorregler über PWM
        wait_ms(20);                                     // Orden al controlador del motor a través de PWM
        abweichung = gyro_angle - winkel_soll;        // Kontrolle der Regelgenauigkeit
                                                        //comprobación de la precisión del control
        if (start == true) 
        {                                             
            leistung_alt = leistung;
            start = false;
        }      
        else 
        {
            if (fabs(winkel) > 5.0) 
            {                                     
                if ( fabs(leistung) > min_leistung) 
                { 
                    float delta_leistung = fabs(leistung) - fabs(leistung_alt);  // Bestimmung Abfall Delta -- Det. Residuo Delta
                    if (delta_leistung < -0.001) 
                    {                                 // Abbremsung des Motors bei Leistungsabnahme
                                                        // Desaceleración del motor al bajar potencia
                        kreisel.write(calcStellwert(Bremsrichtung * min_leistung)); // direcc frenado * min_potencia
                        wait_ms(10);
                    }
                }
            }
            
            leistung_alt = leistung;                            //la pot anterior pasa a ser la nueva
        }                                                       // Ende des Bremsverfahrens -- Fin proced. frenado
        
        if (fabs(abweichung) <= regelgenauigkeit) 
        {             // Vorzeichen für Bremsung wird bestimmt -- Determ. señal de frenado
            print_gyro_data();                                  // Funktionsaufruf Gyrodaten ausgeben 
                                                                    // Llamada a la funcion de salida de datos del giroscopio
            kreisel.write(calcStellwert(Bremsrichtung * 100));  // Vollbremsung -- Frenado completo
            wait_ms(650); 
            kreisel.write(calcStellwert(0));                    // Motor aus -- Apagado del motor
        }
    }
       
    float end_angle = gyro_angle;        //vuelve a leer el ángulo antes de acabar la interrupcion para observar el cambio.
    ios.printf("Plattform wurde gedreht um %+6.1f deg\n\n", end_angle - start_angle);
                        // "La plataforma se giro alrededor de x grados"
}
 
 
///////////////////////////////////////////////////////////////////////////////////////////////////
// LageregelungAchseDyn // FUNKTION ZUR LAGEREGELUNG FÜR EINE ACHSE - DYNAMISCH - PID Regler
                            // FUNCIÓN DE CONTROL DE POSICIÓN PARA UN EJE - DINÁMICO - 
void LageregelungAchseDyn(float kp, float kd, float ki, float winkel, const float& gyro_veloc, 
                          const float& gyro_angle, PwmOut& kreisel, float& dyn_leistung_leerlauf)
{                                                                      // din_potencia_circuito_abierto
    
    ios.printf("Start LageregelungAchseDyn\n");  //posicion inicial control_eje_dinamico
    float start_angle = gyro_angle;
    float winkel_soll = gyro_angle + winkel;
    float abweichung = gyro_angle - winkel_soll;
    float abweichung_sum = abweichung;           //suma de desviacion
    float leistung_alt = dyn_leistung_leerlauf;  //potencia anterior = din_potenc_circabierto
    bool  start = true;
    float leistung = 0.0;
    float i_glied = 0.0;     //glied == eslabon, miembro
    
    int cnt = 0; 
    while (fabs(abweichung) > regelgenauigkeit)  //si la desviacion es mayor que la variable de control
    {
        cnt += 1;
        i_glied = abweichung_sum * 0.001 * ki;  //integrador con la suma de desviaciones
        ios.printf("%f\n", i_glied);
        if (i_glied > 0.5) i_glied = 0.5;                       // I-Glied Limiter 
        if (i_glied < -0.5) i_glied = -0.5;
        // Sumierstelle (PID Glieder) --  Punto de suma, (enlaces PID)
        leistung = dyn_leistung_leerlauf + abweichung * kp - gyro_veloc * kd + i_glied;            
        if (leistung > max_leistung) leistung = max_leistung;
        if (leistung < min_leistung) leistung = min_leistung;
        int Bremsrichtung = -1;                                 // Vorzeichen der Abbremsrichtung -- signo direcc frenado
 
        float Stellwert = calcStellwert(leistung);              // Berechnung Stellwert
        kreisel.write(Stellwert);                               // Befehl an Motorregler über Pwm
        wait_ms(10);       
    
        if (start == true) 
        {                                   
            leistung_alt = leistung;
            start = false;
        }     
        /*else 
        {                       
            if (leistung > min_leistung) 
            { 
                float delta_leistung = leistung - leistung_alt; // Bestimmung Abfall Delta
                
                // bei Abnahme der Leistung - Motor abbremsen
                if (delta_leistung < -0.0001) 
                {                 
                    kreisel.write(calcStellwert(Bremsrichtung * min_leistung)); 
                    wait_ms(5);
                }
            }
        }
        */
        leistung_alt = leistung;
        abweichung = gyro_angle - winkel_soll;
        abweichung_sum += abweichung;
    }
    
    dyn_leistung_leerlauf = leistung - i_glied;
    if (dyn_leistung_leerlauf < min_leistung) dyn_leistung_leerlauf = min_leistung;
    if (dyn_leistung_leerlauf > max_leistung) dyn_leistung_leerlauf = max_leistung;
    ios.printf("Leerlaufleistung: %f\n", dyn_leistung_leerlauf);  
    float end_angle = gyro_angle;      
    ios.printf("Plattform wurde gedreht um %+6.1f deg\n\n", end_angle - start_angle);
    // Counter für durchlaufene Regelzyklen             
    ios.printf("\nEnde LageregelungAchseDyn; Counter: %i\n\n", cnt);                               
}
 
 
///////////////////////////////////////////////////////////////////////////////////////////////////
// main // Hauptprogramm
 
int main() 
{                                                          
    ios.baud(115200);                                                // Baudrate XBee Funkmodul --XBee modulo de radio
    um6_uart.baud(115200);                                           // Baudrate UM6-lt 
    um6_uart.attach(&rxCallback, MODSERIAL::RxIrq);                  // Interrupt Funktion für UART
 
    rst = 1;                                                         // Reset-Pin von XBee auf ON
    
    ios.printf("\nBitte warten, Startvorgang der Plattform...\n\n"); // Start-UP Prozedur -- Procedimiento de comienzo
    um6_uart.putc(0xAC);                                             // Nulliert die Rate-Gyros -- pone a cero los giroscopos
    wait_ms(3500);
    um6_uart.putc(0xAC);                                             // Nulliert die Rate-Gyros -- pone a cero los giroscopos
    wait_ms(3500);
    um6_uart.putc(0xAC);                                             // Nulliert die Rate-Gyros -- pone a cero los giroscopos
    wait_ms(3500);
 
    x_kreisel.period_ms(pulsweite);              //pulsweite -- ancho de pulso                     
    y_kreisel.period_ms(pulsweite);
    z_kreisel.period_ms(pulsweite);
    
    ios.printf("\n\nTESTPLATTFORM ZUR SATELLITENLAGEREGELUNG MIT REAKTIONSKREISELN:\n\n");
                        // CONTROL DE POSICIÓN DEL SATÉLITE CON GIROSCOPIOS DE REACCIÓN
    int choice1 = 0;
    
    do 
    {
        do 
        {                   // Por favor, siga las siguientes instrucciones, si no lo ha hecho ya.
            ios.printf("\n\nBitte folgende Anweisungen ausfuehren,falls nicht bereits erfolgt!\n"
               "Stromversorgung der Testplattform trennen und ohne Druckluft mit X-Achse auf mag."
               " Norden ausrichten.\n"
               // Desconectar la alimentación de la plataforma de pruebas y alinear al norte magnético con el eje X sin aire comprimido
               "Testplattform mit Wasserwaage horizontal tarieren.\n"
               // Tarar la plataforma de pruebas horizontalmente con un nivel de burbuja
               "Danach Stromversorgung herstellen und mbed Reset-Taste betaetigen.\n\n");
               // A continuación, conecte la fuente de alimentación y pulse el botón de reinicio del mbed.

            
            x_kreisel.write(calcStellwert(0));                       //Initialisierung X-Motorrelger 
            y_kreisel.write(calcStellwert(0));                       //Initialisierung Y-Motorrelger
            z_kreisel.write(calcStellwert(0));                       //Initialisierung Z-Motorrelger

            /* En esta inicialización lo que está haciendo es definir el estado INICIAL de la cama, es decir
             está definiendo el cero de todos los ejes escribiendo en la función de calcStellwert, que es la que
             determina la precisión de la posición, determinando así el balance exacto del sistema.

             Aquí es donde usa el nivel de burbuja circular, para decirle al sistema exactamente cómo debe estar balanceado.
            */

            ios.printf("Menueauswahl:\n");         //menú de selección                 
            ios.printf("\t1 - Testplattform um einzelne Achsen drehen\n"); 
                        // Rotación de la plataforma de pruebas alrededor de ejes individuales
            ios.printf("\t2 - Testplattform um drei Achsen drehen\n");
                        // Rotación de la plataforma de pruebas alrededor de tres ejes
            ios.printf("\t3 - Lage der Testplattform aktiv regeln fuer dynamische Fehler\n");
                        // Controlar activamente la posición de la plataforma de pruebas para los fallos dinámicos
            ios.printf("\t4 - Gyrodaten ausgeben [Winkel/Winkelgeschwindigkeiten]\n");
                        // Datos del giroscopio de salida [Ángulo/velocidades angulares].
            ios.printf("\t5 - Programm beenden\n");
                        // Salir del programa
            ios.printf("\nEingabe >"); // Entre en >
            ios.scanf("%i", &choice1);  
                // en esta instrucción es donde pide el número a elegir. En nuestro caso tendremos que decirle que pida el número
                // del comando al IRremote, por donde tendremos el envío de interrupciones.
        }while (choice1 < 1 || choice1 > 5);
        
        if (choice1 == 1) // Rotación de la plataforma de pruebas alrededor de ejes individuales
        { 
            ios.printf("\nTestplattform ist initialisiert.\n\n");    // Inicio de la cama
            print_gyro_angles();                                     // Funktionsaufruf Gyrodwinkel ausgeben     
            float x_kp = x_kp_def;                                       // Llamada a la función Ángulo de salida del giroscopio
            float x_kd = x_kd_def;
            float x_winkel = 0.0;
            float y_kp = y_kp_def; 
            float y_kd = y_kd_def;
            float y_winkel = 0.0;
            float z_kp = z_kp_def; 
            float z_kd = z_kd_def;
            float z_winkel = 0.0;
 
            int choice2 = 0;
            
            do 
            {
                do 
                {
                    ios.printf("\n\nLAGEREGELUNG EINZELNER ACHSEN:\n\n");  
                                    // CONTROL DE POSICIÓN DE EJES INDIVIDUALES                       
                    ios.printf("Menueauswahl:\n"); // Selección de menu
                    ios.printf("\t1 - Satellitenplattform rollen (um X-Achse drehen)\n");
                                        // Rodar la plataforma del satélite (girar alrededor del eje X)
                    ios.printf("\t2 - Satellitenplattform nicken (um Y-Achse drehen)\n");
                                        // Plataforma de satélites Nod (girar alrededor del eje Y)
                    ios.printf("\t3 - Satellitenplattform gieren (um Z-Achse drehen)\n");
                                        // Giro de la plataforma del satélite (rotación alrededor del eje Z)
                    ios.printf("\t4 - Auswahl abbrechen (esc).\n");
                                        // Cancelar la selección
                    ios.printf("\nEingabe >");
                    ios.scanf("%i", &choice2); 
                }while (choice2 < 1 || choice2 > 4);
 
                if (choice2 == 1) // Rodar la plataforma del satélite (girar alrededor del eje X)
                {
                    ios.printf("\n Reglerparameter kp eingeben [0 - +3] >"); //Introduzca el parámetro del regulador kp [0 - +3].
                    ios.scanf("%f", &x_kp);          
                    if (x_kp > x_kp_max) x_kp = x_kp_max;                 // Begrenzung X-Achse kp Pos. -- Limitación eje X positivo
                    if (x_kp < x_kp_min) x_kp = x_kp_min;                 // Begrenzung X-Achse kp Neg. -- Limitación eje X negativo
                    ios.printf("\n Reglerparameter kd eingeben [0 - +3] >"); //Introduzca el parámetro del regulador kd [0 - +3].
                    ios.scanf("%f", &x_kd); 
                    if (x_kd > x_kd_max) x_kd = x_kd_max;                 // Begrenzung X-Achse kd Pos.
                    if (x_kd < x_kd_min) x_kd = x_kd_min;                 // Begrenzung X-Achse kd Neg.      
                    ios.printf("\n Rollwinkel Phi eingeben [-45 - +45] >"); // Introduzca el ángulo de balanceo Phi [-45 - +45].
                    ios.scanf("%f", &x_winkel);
                    if (x_winkel > x_winkel_max) x_winkel = x_winkel_max; // Begrenzung X-Achse Winkel Pos.
                    if (x_winkel < x_winkel_min) x_winkel = x_winkel_min; // Begrenzung X-Achse Winkel Neg.   
                    if (fabs(x_winkel) < regelgenauigkeit) ios.printf("\nRegelung ist nicht notwendig!");
                                                                            // La regulación no es necesaria.
                    ios.printf("\nParameter kp = %f, Parameter kd = %f, Rollwinkel = %f\n", 
                               x_kp, x_kd, x_winkel); 
                    print_gyro_angles();
                    LageregelungAchse(x_kp, x_kd, x_winkel, data.Gyro_Proc_X, data.Roll, x_kreisel);
                }
                else if (choice2 == 2) // Plataforma de satélites Nod (girar alrededor del eje Y)
                {                     //parametros del controlador
                    ios.printf("\n Reglerparameter kp eingeben [0 - +3] >");
                    ios.scanf("%f", &y_kp);
                    if (y_kp > y_kp_max) y_kp = y_kp_max;                 // Begrenzung Y-Achse kp Pos.
                    if (y_kp < y_kp_min) y_kp = y_kp_min;                 // Begrenzung Y-Achse kp Neg.
                    ios.printf("\n Reglerparameter kd eingeben [0 - +3] >");
                    ios.scanf("%f", &y_kd);
                    if (y_kd > y_kd_max) y_kd = y_kd_max;                 // Begrenzung Y-Achse kd Pos.
                    if (y_kd < y_kd_min) y_kd = y_kd_min;                 // Begrenzung Y-Achse kd Neg.
                    ios.printf("\n Nickwinkel Theta eingeben [-45 - +45] >");
                    ios.scanf("%f", &y_winkel);                             
                    if (y_winkel > y_winkel_max) y_winkel = y_winkel_max; // Begrenzung Y-Achse Winkel Pos.
                    if (y_winkel < y_winkel_min) y_winkel = y_winkel_min; // Begrenzung Y-Achse Winkel Neg.   
                    if (fabs(y_winkel) < regelgenauigkeit) ios.printf("\nRegelung ist nicht notwendig!");
                                                                            // La regulación no es necesaria.
                    ios.printf("\nParameter kp = %f, Parameter kd = %f, Nickwinkel = %f\n", 
                                                                        // Angulo de inclinación
                                y_kp, y_kd, y_winkel); 
                    print_gyro_angles();                                                  
                    LageregelungAchse(y_kp, y_kd, y_winkel, data.Gyro_Proc_Y, data.Pitch, y_kreisel);
                        // Posición controlAxis
                 } 
                 else if (choice2 == 3) // Giro de la plataforma del satélite (rotación alrededor del eje Z)
                 {                   // Parámetros del controlador 
                    ios.printf("\n Reglerparameter kp eingeben [0 - +3] >");
                    ios.scanf("%f", &z_kp);
                    if (z_kp > z_kp_max) z_kp = z_kp_max;                 // Begrenzung Z-Achse kp Pos.
                    if (z_kp < z_kp_min) z_kp = z_kp_min;                 // Begrenzung Z-Achse kp Neg.
                    ios.printf("\n Reglerparameter kd eingeben [0 - +3] >");
                    ios.scanf("%f", &z_kd);
                    if (z_kd > z_kd_max) z_kd = z_kd_max;                 // Begrenzung Z-Achse kd Pos.
                    if (z_kd < z_kd_min) z_kd = z_kd_min;                 // Begrenzung Z-Achse kd Neg.   
                    ios.printf("\n Gierwinkel Psi eingeben [-90 - +90] >");
                    ios.scanf("%f", &z_winkel);      
                    if (z_winkel > z_winkel_max) z_winkel = z_winkel_max; // Begrenzung Z-Achse Winkel Pos.
                    if (z_winkel < z_winkel_min) z_winkel = z_winkel_min; // Begrenzung Z-Achse Winkel Neg.   
                    if (fabs(z_winkel) < regelgenauigkeit) ios.printf("\nRegelung ist nicht notwendig!");
                    ios.printf("\nParameter kp = %f, Parameter kd = %f, Gierwinkel = %f\n", 
                                z_kp, z_kd, z_winkel); 
                    print_gyro_angles();
                    LageregelungAchse(z_kp, z_kd, z_winkel, data.Gyro_Proc_Z, data.Yaw, z_kreisel);
                }
            }while(choice2 != 4);
            
        }
        else if (choice1 == 2) // Rotación de la plataforma de pruebas alrededor de tres ejes
        {
            float x_kp = x_kp_def;
            float x_kd = x_kd_def;
            float x_winkel = 0.0;
            float y_kp = y_kp_def; 
            float y_kd = y_kd_def;
            float y_winkel = 0.0;
            float z_kp = z_kp_def; 
            float z_kd = z_kd_def;
            float z_winkel = 0.0;
       
            int choice3 = 0;
            
            do 
            {
                do 
                {                   // AGREGACIÓN DE TODOS LOS EJES
                    ios.printf("\n\nLAGEREGELUNG ALLER ACHSEN:\n\n");
                    ios.printf("Bitte Plattform in gewuenschte Ausgangsposition bringen.\n");
                                // Por favor, ponga la plataforma en la posición inicial deseada
                    ios.printf("Plattform wird in Z-Y-X Reihenfolge gedreht!\n\n");
                                // La plataforma gira en el orden Z-Y-X.
                    ios.printf("Aktuelle Werte fuer Regler X-Achse: kp = %f kd = %f\n\n", x_kp, x_kd); // valores actuales para...
                    ios.printf("Aktuelle Werte fuer Regler Y-Achse: kp = %f kd = %f\n\n", y_kp, y_kd);
                    ios.printf("Aktuelle Werte fuer Regler Z-Achse: kp = %f kd = %f\n\n", z_kp, z_kd);
                    ios.printf("Menueauswahl:\n");
                    ios.printf("\t1 - Regelparameter aendern?\n"); //¿Cambiar parámetros de control?
                    ios.printf("\t2 - Plattform drehen-Winkel eingeben\n"); // Ingrese el ángulo para el giro de la plataforma
                    ios.printf("\t3 - Auswahl abbrechen (esc).\n");
                    ios.printf("\nEingabe >");
                    ios.scanf("%i", &choice3); 
                }while (choice3 < 1 || choice3 > 3);
                      
                if (choice3 == 1)
                {
                    ios.printf("\n Reglerparameter x_kp eingeben [0 - +3] >");
                    ios.scanf("%f", &x_kp);          
                    if (x_kp > x_kp_max) x_kp = x_kp_max;                 // Begrenzung X-Achse kp Pos.
                    if (x_kp < x_kp_min) x_kp = x_kp_min;                 // Begrenzung X-Achse kp Neg.
                    ios.printf("\n Reglerparameter x_kd eingeben [0 - +3] >");
                    ios.scanf("%f", &x_kd); 
                    if (x_kd > x_kd_max) x_kd = x_kd_max;                 // Begrenzung X-Achse kd Pos.
                    if (x_kd < x_kd_min) x_kd = x_kd_min;                 // Begrenzung X-Achse kd Neg.

                    ios.printf("\n Reglerparameter y_kp eingeben [0 - +3] >");
                    ios.scanf("%f", &y_kp);
                    if (y_kp > y_kp_max) y_kp = y_kp_max;                 // Begrenzung Y-Achse kp Pos.
                    if (y_kp < y_kp_min) y_kp = y_kp_min;                 // Begrenzung Y-Achse kp Neg.
                    ios.printf("\n Reglerparameter y_kd eingeben [0 - +3] >");
                    ios.scanf("%f", &y_kd);
                    if (y_kd > y_kd_max) y_kd = y_kd_max;                 // Begrenzung Y-Achse kd Pos.
                    if (y_kd < y_kd_min) y_kd = y_kd_min;                 // Begrenzung Y-Achse kd Neg.

                    ios.printf("\n Reglerparameter z_kp eingeben [0 - +3] >");
                    ios.scanf("%f", &z_kp);
                    if (z_kp > z_kp_max) z_kp = z_kp_max;                 // Begrenzung Z-Achse kp Pos.
                    if (z_kp < z_kp_min) z_kp = z_kp_min;                 // Begrenzung Z-Achse kp Neg.
                    ios.printf("\n Reglerparameter z_kd eingeben [0 - +3] >");
                    ios.scanf("%f", &z_kd);
                    if (z_kd > z_kd_max) z_kd = z_kd_max;                 // Begrenzung Z-Achse kd Pos.
                    if (z_kd < z_kd_min) z_kd = z_kd_min;                 // Begrenzung Z-Achse kd Neg. 
                    ios.printf("\n\n"); 
                    ios.printf("Aktuelle Werte fuer Regler X-Achse: kp = %f kd = %f\n\n", x_kp, x_kd);
                    ios.printf("Aktuelle Werte fuer Regler Y-Achse: kp = %f kd = %f\n\n", y_kp, y_kd);
                    ios.printf("Aktuelle Werte fuer Regler Z-Achse: kp = %f kd = %f\n\n", z_kp, z_kd);
                }
                else if (choice3 == 2)  // Ingrese el ángulo para el giro de la plataforma
                {                                                         // Eingabe der Drehwinkel -- Ingrese ángulo de rotación
                    ios.printf("\n Gierwinkel Psi eingeben [-90 - +90] >");
                    ios.scanf("%f", &z_winkel);      
                    if (z_winkel > z_winkel_max) z_winkel = z_winkel_max; // Begrenzung Z-Achse Winkel Pos.
                    if (z_winkel < z_winkel_min) z_winkel = z_winkel_min; // Begrenzung Z-Achse Winkel Neg.
                    ios.printf("\n Nickwinkel Theta eingeben [-45 - +45] >");
                    ios.scanf("%f", &y_winkel);                             
                    if (y_winkel > y_winkel_max) y_winkel = y_winkel_max; // Begrenzung Y-Achse Winkel Pos.
                    if (y_winkel < y_winkel_min) y_winkel = y_winkel_min; // Begrenzung Y-Achse Winkel Neg.
                    ios.printf("\n Rollwinkel Phi eingeben [-45 - +45] >");
                    ios.scanf("%f", &x_winkel);
                    if (x_winkel > x_winkel_max) x_winkel = x_winkel_max; // Begrenzung X-Achse Winkel Pos.
                    if (x_winkel < x_winkel_min) x_winkel = x_winkel_min; // Begrenzung X-Achse Winkel Neg.
                        
                    if (fabs(z_winkel) < regelgenauigkeit && 
                        fabs(y_winkel) < regelgenauigkeit && 
                        fabs(x_winkel) < regelgenauigkeit)
                    {
                        ios.printf("\nRegelung ist nicht notwendig!"); // La regulación no es necesaria.
                    }
                    else 
                    {
                        print_gyro_angles();
                        LageregelungAchse(z_kp, z_kd, z_winkel, data.Gyro_Proc_Z, data.Yaw, z_kreisel);
                        wait_ms(250);   
                        LageregelungAchse(y_kp, y_kd, y_winkel, data.Gyro_Proc_Y, data.Pitch, y_kreisel);
                        wait_ms(250);
                        LageregelungAchse(x_kp, x_kd, x_winkel, data.Gyro_Proc_X, data.Roll, x_kreisel);
                        print_gyro_angles();
                        ios.printf("\n\nSoll die Plattform wieder in ihre Ausgangslage gefahren werden? "
                                   "ja/nein\n"); //¿Se debe volver a mover la plataforma a su posición inicial?
                        char yes_no[50]; 
                        ios.scanf("%s", yes_no);
                        
                        if (strcmp(yes_no, "ja") == 0)    // Plattform in auf Ausgangsposition drehen
                        {                                   // Gire la plataforma a la posición inicial.
                            LageregelungAchse(x_kp, x_kd, -x_winkel, data.Gyro_Proc_X, data.Roll, x_kreisel);
                            wait_ms(250);  
                            LageregelungAchse(y_kp, y_kd, -y_winkel, data.Gyro_Proc_Y, data.Pitch, y_kreisel);
                            wait_ms(250);
                            LageregelungAchse(z_kp, z_kd, -z_winkel, data.Gyro_Proc_Z, data.Yaw, z_kreisel);
                            print_gyro_angles();
                        }
                    }
                }  
                            
            }while(choice3 != 3);
        }
        else if (choice1 == 3) // Controlar activamente la posición de la plataforma de pruebas para los fallos dinámicos
        {
            float x_kp = x_kp_def;
            float x_kd = x_kd_def;
            float x_ki = x_ki_def;
            float y_kp = y_kp_def; 
            float y_kd = y_kd_def;
            float y_ki = y_ki_def;
            float z_kp = z_kp_def; 
            float z_kd = z_kd_def;
            float z_ki = z_ki_def;
       
            int choice4 = 0;
            
            do 
            {
                do 
                {                  // Regule activamente la ubicación de la plataforma de prueba para detectar errores dinámicos     
                    ios.printf("\n\nLage der Testplattform aktiv regeln fuer dynamische Fehler" 
                               " (Position halten):\n\n"); // Mantén la posición
                                    // Lleve la plataforma a la posición inicial deseada y manténgala...
                    ios.printf("\n\nBitte Plattform in gewuenschte Ausgangsposition bringen und halten "
                               "bis die Kreisel " // ...en el disco...
                               "ihre Leerlaufdrehzahl erreicht haben.\n\n"); // ...se ha alcanzado su velocidad de ralentí.
                    ios.printf("Aktuelle Werte fuer Regler X-Achse: kp = %f kd = %f ki = %f\n", 
                                x_kp, x_kd, x_ki); 
                    ios.printf("Aktuelle Werte fuer Regler Y-Achse: kp = %f kd = %f ki = %f\n", 
                                y_kp, y_kd, y_ki);
                    ios.printf("Aktuelle Werte fuer Regler Z-Achse: kp = %f kd = %f ki = %f\n\n", 
                                z_kp, z_kd, z_ki);
                    ios.printf("Menueauswahl:\n");
                    ios.printf("\t1 - Regelparameter aendern?\n"); // solo lo tiene activo para el eje Z
                    ios.printf("\t2 - Plattform aktivieren\n");
                    ios.printf("\t3 - Auswahl abbrechen (esc).\n");
                    ios.printf("\nEingabe >");
                    ios.scanf("%i", &choice4); 
                }while (choice4 < 1 || choice4 > 3);  
                       
                if (choice4 == 1)
                {
                    /*
                    ios.printf("\n Reglerparameter x_kp eingeben [0 - +3] >");
                    ios.scanf("%f", &x_kp);          
                    if (x_kp > x_kp_max) x_kp = x_kp_max;                 // Begrenzung X-Achse kp Pos.
                    if (x_kp < x_kp_min) x_kp = x_kp_min;                 // Begrenzung X-Achse kp Neg.
                    ios.printf("\n Reglerparameter x_kd eingeben [0 - +3] >");
                    ios.scanf("%f", &x_kd); 
                    if (x_kd > x_kd_max) x_kd = x_kd_max;                 // Begrenzung X-Achse kd Pos.
                    if (x_kd < x_kd_min) x_kd = x_kd_min;                 // Begrenzung X-Achse kd Neg.
                    ios.printf("\n Reglerparameter x_ki eingeben [0 - +0.5] >");
                    ios.scanf("%f", &x_ki); 
                    if (x_ki > x_ki_max) x_ki = x_ki_max;                 // Begrenzung X-Achse ki Pos.
                    if (x_ki < x_ki_min) x_ki = x_ki_min;                 // Begrenzung X-Achse ki Neg.
                    ios.printf("\n Reglerparameter y_kp eingeben [0 - +3] >");
                    ios.scanf("%f", &y_kp);
                    if (y_kp > y_kp_max) y_kp = y_kp_max;                 // Begrenzung Y-Achse kp Pos.
                    if (y_kp < y_kp_min) y_kp = y_kp_min;                 // Begrenzung Y-Achse kp Neg.
                    ios.printf("\n Reglerparameter y_kd eingeben [0 - +3] >");
                    ios.scanf("%f", &y_kd);
                    if (y_kd > y_kd_max) y_kd = y_kd_max;                 // Begrenzung Y-Achse kd Pos.
                    if (y_kd < y_kd_min) y_kd = y_kd_min;                 // Begrenzung Y-Achse kd Neg.
                    ios.printf("\n Reglerparameter y_ki eingeben [0 - +0.5] >");
                    ios.scanf("%f", &y_ki); 
                    if (y_ki > y_ki_max) y_ki = y_ki_max;                 // Begrenzung Y-Achse ki Pos.
                    if (y_ki < y_ki_min) y_ki = y_ki_min;                 // Begrenzung Y-Achse ki Neg.
                    */  
                    ios.printf("\n Reglerparameter z_kp eingeben [0 - +3] >");
                    ios.scanf("%f", &z_kp);
                    if (z_kp > z_kp_max) z_kp = z_kp_max;                 // Begrenzung Z-Achse kp Pos.
                    if (z_kp < z_kp_min) z_kp = z_kp_min;                 // Begrenzung Z-Achse kp Neg.
                    ios.printf("\n Reglerparameter z_kd eingeben [0 - +3] >");
                    ios.scanf("%f", &z_kd);
                    if (z_kd > z_kd_max) z_kd = z_kd_max;                 // Begrenzung Z-Achse kd Pos.
                    if (z_kd < z_kd_min) z_kd = z_kd_min;                 // Begrenzung Z-Achse kd Neg. 
                    ios.printf("\n Reglerparameter z_ki eingeben [0 - +0.5] >");
                    ios.scanf("%f", &z_ki); 
                    if (z_ki > z_ki_max) z_ki = z_ki_max;                 // Begrenzung Z-Achse ki Pos.
                    if (z_ki < z_ki_min) z_ki = z_ki_min;                 // Begrenzung Z-Achse ki Neg.
                    ios.printf("\n\n"); 
                    //ios.printf("Aktuelle Werte fuer Regler X-Achse: kp = %f kd = %f ki = %f\n\n", 
                    //            x_kp, x_kd, x_ki); 
                    //ios.printf("Aktuelle Werte fuer Regler Y-Achse: kp = %f kd = %f ki = %f\n\n", 
                    //            y_kp, y_kd, y_ki);
                    ios.printf("Aktuelle Werte fuer Regler Z-Achse: kp = %f kd = %f ki = %f\n\n", 
                                z_kp, z_kd, z_ki);
                }
                else if (choice4 == 2)  // Activar la plataforma
                {                                       
                    ios.printf("\n\nUm abzubrechen bitte Reset-Taste an mbed betaetigen!\n\n");
                                    // Para cancelar, presione el botón de reinicio en mbed.
                    float ziel_x_winkel = data.Roll;
                    float ziel_y_winkel = data.Pitch;
                    float ziel_z_winkel = data.Yaw;
                    
                    print_gyro_angles();                                 
                    float x_dyn_leistung_leerlauf = (min_leistung + max_leistung)/2; // Leistung im Leerlauf -- Potencia en vacío
                    float y_dyn_leistung_leerlauf = (min_leistung + max_leistung)/2; // Leistung im Leerlauf 
                    float z_dyn_leistung_leerlauf = (min_leistung + max_leistung)/2; // Leistung im Leerlauf
                    
                    wait_ms(2000);
                    z_kreisel.write(calcStellwert(z_dyn_leistung_leerlauf));
                    //y_kreisel.write(calcStellwert(y_dyn_leistung_leerlauf));
                    //x_kreisel.write(calcStellwert(x_dyn_leistung_leerlauf));
            
                    while(1)
                    {
                        while (fabs(ziel_z_winkel - data.Yaw) > regelgenauigkeit)
                        {     
                            LageregelungAchseDyn(z_kp, z_kd, z_ki, ziel_z_winkel - data.Yaw, data.Gyro_Proc_Z, 
                                                 data.Yaw, z_kreisel, z_dyn_leistung_leerlauf);
                        }
                        
                        /*
                        while (fabs(ziel_y_winkel - data.Pitch) > regelgenauigkeit)
                        {  
                            LageregelungAchseDyn(y_kp, y_kd, y_ki, ziel_y_winkel - data.Roll, data.Gyro_Proc_Y, 
                                                 data.Pitch, y_kreisel, y_dyn_leistung_leerlauf);  
                        }
                        
                        while (fabs(ziel_x_winkel - data.Roll) > regelgenauigkeit)
                        {
                            LageregelungAchseDyn(x_kp, x_kd, x_ki, ziel_x_winkel - data.Pitch, data.Gyro_Proc_X, 
                                                 data.Roll, x_kreisel, x_dyn_leistung_leerlauf);
                        }
                        */
                    }
                }
            }while(choice4 != 3);
        }
        else if (choice1 == 4)
        { 
            print_gyro_data();
        }
          
    }while(choice1 != 5); 
 
    ios.printf("\n\nProgramm beendet.\n");
}