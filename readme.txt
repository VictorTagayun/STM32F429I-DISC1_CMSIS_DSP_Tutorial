used X-CUBE-DSPDEMO as basis

1. Download orig X-CUBE-DSPDEMO and overwrite with this zip file to copy some modifications I did on the source files
2. First tested on STM32F429I-DISC1_FIR_FFT_wth_Print
>> import by ioc and generate code first
>> click "Use float with printf ...."
>> add libarm_cortexM4lf_math.a in the linker settings
3. add STM32F429I-DISC1_FIR_FFT_wth_Print_LCD_off for LDC off
4. other projects like single data, etc. port to G474, F769 and compare timing

