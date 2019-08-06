# House Price Prediction On STM32
Simple STM32 example showing how to use the STM32 AI API with a custom application that use a NN that predict price of houses (just very simple NN for the app).
You can just take the sources to get inspired and create you own AI project

## Neural network
- The NN is very simple taking two float number as input: (number of room, surface)
- Output an estimation of the price depending on the data it was trained on
- The NN is just a very simple example, badly trained, with not enouch sample data... The goal was to just get an artificiel neural net to start an STM32 project

## Project
- This is an IAR IDE Project 
- Code generated for an STM32L746 and used with an STM32L746 discovery board
- You need to add CMSIS + HAL (Use CubeMX to regenerate for the platform you want to use)
- Python code used to generate the Keras NN included 
- The code was generated using CubeMX and X-Cube-AI v3.3.0 (old version)

## Have fun !
- Have fun developing you own STM32 AI projects !
