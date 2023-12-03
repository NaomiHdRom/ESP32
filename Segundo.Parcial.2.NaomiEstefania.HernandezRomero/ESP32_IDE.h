// UNIVERSIDAD NACIONAL AUTÓNOMA DE MÉXICO
// FACULAD DE INGENIERÍA
// DEPARAMENTO DE MECATRÓNICA

// AUTORES:   M.I. ULISES M. PEÑUELAS RIVAS
//            M.I. YUKIHIRO MINAMI KOYAMA
//
// Versión 2021.2M

/*********************************************************************
 Todos los derechos reservados. Facultad de Ingeniería de la 
 Universidad Nacional Autónoma de México © 2021. Queda estrictamente 
 prohibidos su uso fuera del ámbito académico, alteración, descarga, 
 difusión o divulgación por cualquier medio, así como su reproducción 
 parcial o total.
********************************************************************/

//Notas de uso de la biblioteca
// El archivo de bibioteca está depurada para funcionar principalmente
// con el microcontrolador ESP32, la tarjeta Arduino AVR ATMega2560 apoya
// a la simulación de la ESP32 sobre el paquete Proteus, en dónde solamente
// se usarán los pines 22-29, 50-55, 57-66 para hacer una interfaz de 
// subcircuito con el gráfico de una tarjeta ESP32 DEV KIT V1.
//
//**********************************************

#if defined(__AVR_ATmega2560__)
  int pld_Ent[12] = {53, 52, 51, 50, 10, 11, 12, 13, 21, 20, 19, 18};
  int pld_Sal[12] = {54, 55, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66};
  bool pld_clk = false;
  #define E0 pld_Ent[0]
  #define E1 pld_Ent[1]
  #define E2 pld_Ent[2]
  #define E3 pld_Ent[3]
  #define E4 pld_Ent[4]
  #define E5 pld_Ent[5]
  #define E6 pld_Ent[6]
  #define E7 pld_Ent[7]
  #define E8 pld_Ent[8]
  #define E9 pld_Ent[9]
  #define E10 pld_Ent[10]
  #define E11 pld_Ent[11]
  #define S0 pld_Sal[0]
  #define S1 pld_Sal[1]
  #define S3 pld_Sal[2]
  #define S4 pld_Sal[3]
  #define S5 pld_Sal[4]
  #define S6 pld_Sal[5]
  #define S7 pld_Sal[6]
  #define S8 pld_Sal[7]
  #define S9 pld_Sal[8]
  #define S10 pld_Sal[9]
  #define S11 pld_Sal[10]
  #define S12 pld_Sal[11]
#endif
//**********************************************

//**********************************************
#if defined(ESP32)
  int pld_Ent[] = {36, 39, 34, 35, 32, 33, 25, 26, 27, 14, 12, 13};
  int pld_Sal[] = {23, 22, 3, 21, 19, 18, 5, 17, 16, 4, 2, 15};
  #define E0 pld_Ent[0]
  #define E1 pld_Ent[1]
  #define E2 pld_Ent[2]
  #define E3 pld_Ent[3]
  #define E4 pld_Ent[4]
  #define E5 pld_Ent[5]
  #define E6 pld_Ent[6]
  #define E7 pld_Ent[7]
  #define E8 pld_Ent[8]
  #define E9 pld_Ent[9]
  #define E10 pld_Ent[10]
  #define E11 pld_Ent[11]
  #define S0 pld_Sal[0]
  #define S1 pld_Sal[1]
  #define S3 pld_Sal[2]
  #define S4 pld_Sal[3]
  #define S5 pld_Sal[4]
  #define S6 pld_Sal[5]
  #define S7 pld_Sal[6]
  #define S8 pld_Sal[7]
  #define S9 pld_Sal[8]
  #define S10 pld_Sal[9]
  #define S11 pld_Sal[10]
  #define S12 pld_Sal[11]
#endif
//**********************************************
