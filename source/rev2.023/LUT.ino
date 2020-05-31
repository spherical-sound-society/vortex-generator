

// rungler clock chanel A, value hsyteresys chanel B
void rungler() {
  digitalWriteFast(Th4Clock_Pin,HIGH);
  th4Counter++;
  if (th4Counter == 4) {
    th4Counter = 0;
    digitalWriteFast(Th4Clock_Pin,LOW);
  }
 // rungleRegister = (rungleRegister << 1)+B_Decay_Value;
  rungleRegister = rungleRegister << 1;
  //Serial.print("rungleRegister:");
  //Serial.println(rungleRegister);

  //TODO sacar esto de rungler y ejecutarlo en el loop principal?
  // looping debe suceder antes o despues de rungleRegister= rungleRegister<< 1;
  //if its looping bitWrite(rungleRegister, 0, bitRead(rungleRegister,7);
  // if rungler is not looping
  if (B_OutputVoltage > 8192 && channelBHysteresys) {
    bitWrite(rungleRegister, 0, 1);
  } else {
    bitWrite(rungleRegister, 0, 0);
  }
  if (B_OutputVoltage < 8192) {
    channelBHysteresys = true;
  }


  // USAR DIGITALWRITEFAST
  digitalWrite (RunglerOutA_Pin, bitRead(rungleRegister, 5));
  digitalWrite (RunglerOutB_Pin, bitRead(rungleRegister, 6));
  digitalWrite (RunglerOutC_Pin, bitRead(rungleRegister, 7));
  //Serial.print(" RunglerOutA_Pin:");
  //Serial.println(bitRead(rungleRegister, 5));

}



/*
  float expoTable[128] = {1.0, 1.0536326, 1.1101418, 1.1696815, 1.2324146, 1.2985122, 1.3681549, 1.4415325, 1.5188457, 1.6003054, 1.686134, 1.7765658, 1.8718476, 1.9722397, 2.078016, 2.1894655, 2.3068924, 2.430617, 2.5609775, 2.6983292, 2.8430479, 2.995528, 3.1561859, 3.3254602, 3.5038135, 3.6917324, 3.8897297, 4.0983458, 4.318151, 4.5497446, 4.793759, 5.050861, 5.321752, 5.607172, 5.9078994, 6.224755, 6.558605, 6.9103603, 7.2809806, 7.6714787, 8.08292, 8.516429, 8.973187, 9.454442, 9.961509, 10.495771, 11.058686, 11.651794, 12.27671, 12.93514, 13.628888, 14.359839, 15.129997, 15.941458, 16.796438, 17.697277, 18.646427, 19.646482, 20.700178, 21.81038, 22.980125, 24.212614, 25.511198, 26.879435, 28.321047, 29.839975, 31.440376, 33.126602, 34.903275, 36.775227, 38.747574, 40.825714, 43.0153, 45.32232, 47.753082, 50.314198, 53.012676, 55.855892, 58.851585, 62.00796, 65.3336, 68.83761, 72.529564, 76.4195, 80.5181, 84.83649, 89.38648, 94.18053, 99.23167, 104.55371, 110.16122, 116.06947, 122.29453, 128.85353, 135.76431, 143.04565, 150.71759, 158.801, 167.31784, 176.29156, 185.74658, 195.70859, 206.20499, 217.26433, 228.91682, 241.19414, 254.13007, 267.75977, 282.12033, 297.25122, 313.19363, 329.9909, 347.68924, 366.3368, 385.98425, 406.68567, 428.49738, 451.47864, 475.6927, 501.2054, 528.0862, 556.40894, 586.2507, 617.69293, 650.8212, 685.72656, 722.504, 761.2535
                       };
  float logTable[128] = {
  796.8609, 756.29846, 717.80084, 681.2632, 646.585, 613.67206, 582.43475, 552.78723, 524.64886, 497.94278, 472.5964, 448.53995, 425.70804, 404.03854, 383.47186, 363.9521, 345.42615, 327.843, 311.15488, 295.3164, 280.284, 266.01678, 252.47592, 239.6242, 227.42668, 215.85004, 204.8628, 194.43471, 184.53745, 175.14409, 166.22878, 157.76727, 149.73657, 142.11456, 134.88054, 128.01482, 121.49852, 115.31391, 109.444145, 103.87317, 98.58574, 93.567474, 88.804665, 84.28426, 79.99398, 75.922066, 72.05745, 68.38954, 64.908325, 61.604332, 58.46852, 55.492313, 52.667614, 49.98669, 47.44224, 45.027313, 42.735302, 40.559967, 38.495365, 36.535847, 34.67608, 32.910973, 31.235722, 29.645746, 28.136696, 26.704468, 25.345144, 24.055008, 22.830547, 21.66841, 20.565434, 19.518602, 18.525053, 17.58208, 16.687109, 15.837689, 15.031512, 14.266367, 13.540174, 12.8509445, 12.196796, 11.57595, 10.986705, 10.427451, 9.896668, 9.392901, 8.914779, 8.460993, 8.030308, 7.621544, 7.2335873, 6.865379, 6.515914, 6.184237, 5.869443, 5.570673, 5.287112, 5.0179844, 4.7625556, 4.520129, 4.2900434, 4.071669, 3.8644104, 3.6677017, 3.4810066, 3.3038144, 3.1356416, 2.9760294, 2.8245418, 2.6807654, 2.5443075, 2.4147956, 2.2918763, 2.1752138, 2.0644898, 1.9594021, 1.8596634, 1.7650018, 1.6751586, 1.5898887, 1.5089593, 1.4321493, 1.3592492, 1.2900599, 1.2243925, 1.1620678, 1.1029155, 1.0467743
  };*/
