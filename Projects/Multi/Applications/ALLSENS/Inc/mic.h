#ifndef __MIC_H
#define __MIC_H

  #define AUDIO_VOLUME_VALUE       64
  #define AUDIO_CHANNELS           2
  #define AUDIO_SAMPLING_FREQUENCY 16000
  #define PCM_AUDIO_IN_SAMPLES     AUDIO_SAMPLING_FREQUENCY/1000

  /* Code for Acoustic Source Localization integration - Start Section */  
  #define M1_EXT_B 0
  #define M4_EXT_B 1
     
  /* Microphone distance */
  #define SIDE     147
  /* Code for Acoustic Source Localization integration - End Section */   

#endif /* __MIC_H */