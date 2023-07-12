# BluePill_Master_buttons
STM32F103C6 programmed as master to send values via SPI if two different buttons are selected.
  **************************************************************************************
  *El siguiente programa envia datos por medio de protocolo SPI si se preciona un boton
  *Si se acciona un boton 1 conectado en el pin PB12 se envia un 0x01
  *Si se acciona un boton 2 conectado en el pin PB13 se envia un 0x02
  *Ademas cada vez que se envia un valor se enciende el LED sobre la placa
  **************************************************************************************
  *The following program sends data via SPI protocol if a button is pressed
   *If a button 1 connected to pin PB12 is activated, a 0x01 is sent
   *If a button 2 connected to pin PB13 is activated, a 0x02 is sent
   *If In addition, every time a value is sent, the LED on the board lights up.
  ***************************************************************************************
