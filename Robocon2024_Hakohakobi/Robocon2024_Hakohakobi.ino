#include "my_bluepad32.h"
#include <ESP32Servo.h>

Servo SC;//SuctionCups:吸盤
#define servoPin 14
#define pomp_INPUT 18//吸盤のポンプのモータのピン番号
#define pomp_DIR 19
//TODO:pwmが使えるピンを調べて定義する
#define dummy_pin_number 1
#define mecanum_R_F_PWM_PIN dummy_pin_number
#define mecanum_R_F_DIR_PIN dummy_pin_number
#define mecanum_R_B_PWM_PIN dummy_pin_number
#define mecanum_R_B_DIR_PIN dummy_pin_number
#define mecanum_L_F_PWM_PIN dummy_pin_number
#define mecanum_L_F_DIR_PIN dummy_pin_number
#define mecanum_L_B_PWM_PIN dummy_pin_number
#define mecanum_L_B_DIR_PIN dummy_pin_number

#define mecanum_R_F_PWM_channel 10
#define mecanum_R_B_PWM_channel 11
#define mecanum_L_F_PWM_channel 12
#define mecanum_L_B_PWM_channel 13
#define mecanum_PWM_frequency 20        //TODO:最適な値にする
#define mecanum_PWM_resolution 16       //TODO:最適な値にする
#define mecanum_PWM_DutyMax_percentage 0.50
//TODO:PWMの解像度は16bitもいるのか？
constexpr uint32_t mecanum_PWM_DutyMax ((1 <<(mecanum_PWM_resolution)) * mecanum_PWM_DutyMax_percentage);

#define ServoGear 30//サーボ側のギアの歯の数
#define ScGear 27   //吸盤側のギアの歯の数

#define SC_DEFAULT_ANGLE 5//吸盤の初期角度
#define SC_UP_ANGLE 120//吸盤がボックスを丁度持ち上げる角度
#define SC_BOX_STORE_ANGLE 180//吸盤を下げてボックスを送る角度

#define CalcGearRatio(Input,Output) (static_cast<float>(Input)/Output) 
constexpr float GearRatio = CalcGearRatio(ServoGear, ScGear);


#define pompStart 1
#define pompStop 0
void PompControl(bool status){
    digitalWrite(pomp_INPUT,status);
    digitalWrite(pomp_DIR,HIGH);
}

enum class MECANUM_movement {
	mecanum_stop = 0,
	mecanum_forward,
	mecanum_backward,
	mecanum_rightward,
	mecanum_leftward,
	mecanum_rightFront_ward,
	mecanum_rightBack_ward,
	mecanum_leftFront_ward,
	mecanum_leftBack_ward
};
void mecanum_control(MECANUM_movement movement);

// Arduino setup function. Runs in CPU 1
void setup() {
    Serial.begin(115200);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    SC.setPeriodHertz(50);// Standard 50hz servo
    SC.attach(servoPin, 500, 2400);

    // GearRatio = CalcGearRatio(ServoGear,ScGear);//ギア比の計算 はこのソースファイルの先頭のグローバル変数をまとめて宣言してるとこでやってある

    //吸盤用ポンプの設定
    pinMode(pomp_INPUT,OUTPUT);
    pinMode(pomp_DIR,OUTPUT);

    //メカナムの設定
    ledcSetup(mecanum_R_F_PWM_channel, mecanum_R_F_PWM_PIN, mecanum_PWM_resolution);
    ledcSetup(mecanum_R_B_PWM_channel, mecanum_R_B_PWM_PIN, mecanum_PWM_resolution);
    ledcSetup(mecanum_L_F_PWM_channel, mecanum_L_F_PWM_PIN, mecanum_PWM_resolution);
    ledcSetup(mecanum_L_B_PWM_channel, mecanum_L_B_PWM_PIN, mecanum_PWM_resolution);
    pinMode(mecanum_R_F_DIR_PIN, OUTPUT);
    pinMode(mecanum_R_B_DIR_PIN, OUTPUT);
    pinMode(mecanum_L_F_DIR_PIN, OUTPUT);
    pinMode(mecanum_L_B_DIR_PIN, OUTPUT);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But it might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();

    // Enables mouse / touchpad support for gamepads that support them.
    // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
    // - First one: the gamepad
    // - Second one, which is a "virtual device", is a mouse.
    // By default, it is disabled.
    BP32.enableVirtualDevice(false);

    PompControl(pompStart);
    SC.write(SC_DEFAULT_ANGLE);
}

// Arduino loop function. Runs in CPU 1.
void loop() {
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    if (dataUpdated) {
        processControllers();
    }

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    if(ButtonData == BUTTON_Y){
        Serial.println("BoxStoring Start");
        //サーボを90度傾ける
        SC.write(SC_UP_ANGLE);
        vTaskDelay(1000);//少し待つ
        Serial.println("Pomp Stopped");
        //PompControl(pompStop);//ポンプ停止
        SC.write(SC_BOX_STORE_ANGLE);
        vTaskDelay(1000);
        PompControl(pompStop);
        vTaskDelay(1000);

        Serial.println("BoxStoring Finished");
    }
    else if(ButtonData == BUTTON_B){
        SC.write(SC_DEFAULT_ANGLE);
        PompControl(pompStart);
        Serial.println("Pomp Started");
    }

    vTaskDelay(1);
    
    //delay(150);
}

//--------------mecanum control func and misc.--------

//stop
#define MECANUM_RightFront_stop() do{\
    ledcWrite(mecanum_R_F_PWM_channel, 0);\
}while(0)
#define MECANUM_RightBack_stop() do{\
    ledcWrite(mecanum_R_B_PWM_channel, 0);\
}while(0)
#define MECANUM_LeftFront_stop() do{\
    ledcWrite(mecanum_L_F_PWM_channel, 0);\
}while(0)
#define MECANUM_LeftBack_stop() do{\
    ledcWrite(mecanum_L_B_PWM_channel, 0);\
}while(0)

// Right Front forward/backward
#define MECANUM_RightFront_forward() do{\
    ledcWrite(mecanum_R_F_PWM_channel, mecanum_PWM_DutyMax);\
    digitalWrite(mecanum_R_F_DIR_PIN, LOW);\
}while(0)
#define MECANUM_RightFront_backward() do{\
    ledcWrite(mecanum_R_F_PWM_channel, mecanum_PWM_DutyMax);\
    digitalWrite(mecanum_R_F_DIR_PIN, HIGH);\
}while(0)

// Right Back
#define MECANUM_RightBack_forward() do{\
    ledcWrite(mecanum_R_B_PWM_channel, mecanum_PWM_DutyMax);\
    digitalWrite(mecanum_R_B_DIR_PIN, LOW);\
}while(0)
#define MECANUM_RightBack_backward() do{\
    ledcWrite(mecanum_R_B_PWM_channel, mecanum_PWM_DutyMax);\
    digitalWrite(mecanum_R_B_DIR_PIN, HIGH);\
}while(0)

// Left Front
#define MECANUM_LeftFront_forward() do{\
    ledcWrite(mecanum_L_F_PWM_channel, mecanum_PWM_DutyMax);\
    digitalWrite(mecanum_L_F_DIR_PIN, LOW);\
}while(0)
#define MECANUM_LeftFront_backward() do{\
    ledcWrite(mecanum_L_F_PWM_channel, mecanum_PWM_DutyMax);\
    digitalWrite(mecanum_L_F_DIR_PIN, HIGH);\
}while(0)

// Left Back
#define MECANUM_LeftBack_forward() do{\
    ledcWrite(mecanum_L_B_PWM_channel, mecanum_PWM_DutyMax);\
    digitalWrite(mecanum_L_B_DIR_PIN, LOW);\
}while(0)
#define MECANUM_LeftBack_backward() do{\
    ledcWrite(mecanum_L_B_PWM_channel, mecanum_PWM_DutyMax);\
    digitalWrite(mecanum_L_B_DIR_PIN, HIGH);\
}while(0)

void ctrl_mecanum(MECANUM_movement movement) {
	switch (movement)
	{
	case MECANUM_movement::mecanum_stop:
		MECANUM_RightFront_stop();
		MECANUM_LeftFront_stop();
		MECANUM_RightBack_stop();
		MECANUM_LeftBack_stop();
		break;
	case MECANUM_movement::mecanum_forward:
		MECANUM_RightFront_forward();
		MECANUM_LeftFront_forward();
		MECANUM_RightBack_forward();
		MECANUM_LeftBack_forward();
		break;
	case MECANUM_movement::mecanum_backward:
		MECANUM_RightFront_backward();
		MECANUM_LeftFront_backward();
		MECANUM_RightBack_backward();
		MECANUM_LeftBack_backward();
		break;
	case MECANUM_movement::mecanum_rightward:
		MECANUM_RightFront_backward();
		MECANUM_LeftFront_forward();
		MECANUM_RightBack_forward();
		MECANUM_LeftBack_backward();
		break;
	case MECANUM_movement::mecanum_leftward:
		MECANUM_RightFront_forward();
		MECANUM_LeftFront_backward();
		MECANUM_RightBack_backward();
		MECANUM_LeftBack_forward();
		break;
	case MECANUM_movement::mecanum_rightFront_ward:
		MECANUM_RightFront_stop();
		MECANUM_LeftFront_forward();
		MECANUM_RightBack_forward();
		MECANUM_LeftBack_stop();
		break;
	case MECANUM_movement::mecanum_rightBack_ward:
		MECANUM_RightFront_backward();
		MECANUM_LeftFront_stop();
		MECANUM_RightBack_stop();
		MECANUM_LeftBack_backward();
		break;
	case MECANUM_movement::mecanum_leftFront_ward:
		MECANUM_RightFront_stop();
		MECANUM_LeftFront_backward();
		MECANUM_RightBack_backward();
		MECANUM_LeftBack_stop();
		break;
	case MECANUM_movement::mecanum_leftBack_ward:
		MECANUM_RightFront_forward();
		MECANUM_LeftFront_stop();
		MECANUM_RightBack_stop();
		MECANUM_LeftBack_forward();
		break;

	default:
		break;
	}
}
