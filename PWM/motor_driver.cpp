#include "L1_Drivers/gpio.hpp"
#include "L1_Drivers/pwm.hpp"
#include "utility/log.hpp"
#include "utility/time.hpp"

#include "L3_Application/oled_terminal.hpp"
#include "L2_HAL/displays/led/onboard_led.hpp"
#include "L2_HAL/switches/button.hpp"

int main(void)
{
	OnBoardLed leds;
	leds.Initialize();
    OledTerminal oled;
    oled.Initialize();

    Button button0(1,19);
    Button button1(1,15);
    Button button2(0,30);
    Button button3(0,29);

    button0.Initialize();
    button1.Initialize();
    button2.Initialize();
    button3.Initialize();

LOG_INFO("Creating Pwm powered Motor (Pwm1,2,3,5)");
//Pwm1:2_0 		Pwm2:2_1	Pwm3:2_2	Pwm5:2_4	
  Pwm p2_0 = Pwm::CreatePwm<1>();
  Pwm p2_1 = Pwm::CreatePwm<2>();
  Pwm p2_2 = Pwm::CreatePwm<3>();
  Pwm p2_5 = Pwm::CreatePwm<5>();
  // Initialize Pwm at 1,000 hz
  uint32_t frequency = 492;//need to look at the data sheet
  float duty = 0.47;//0.47 offset, will be different with different brand of motor, need to look at the data sheet
  p2_0.Initialize(frequency);
  p2_1.Initialize(frequency);
  p2_2.Initialize(frequency);
  p2_5.Initialize(frequency);

  p2_0.SetFrequency(frequency);
   p2_1.SetFrequency(frequency);
    p2_2.SetFrequency(frequency);
     p2_5.SetFrequency(frequency);

//Test first PWM output
  while (true)
  {
  	//adjust the duty, the range should be 0 < duty <1
      if(button0.Pressed()){
      	leds.Toggle(0);
      	duty = duty + 0.05;
      	if(duty > 1){
      		duty = 0.99;
      	}
      }
      else if(button1.Pressed()){
      	leds.Toggle(1);
      	duty = duty - 0.05;
      	if(duty < 0.47){
      		duty = 0.47;
      	}
      }
      else if(button2.Pressed()) {
      	leds.Toggle(2);
      	duty = 0.47;
      }
      else{
      	duty = duty;
      }

      p2_0.SetDutyCycle(duty);
       // p2_1.SetDutyCycle(0.70);
       //  p2_2.SetDutyCycle(0.80);
       //   p2_5.SetDutyCycle(0.99);
      Delay(duty*20);
  }
  return 0;
}
