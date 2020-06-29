#include "mbed.h"
#include "mbed_rpc.h"
#include "bbcar.h"

Thread thread1;
Thread thread2;
EventQueue queue(32 * EVENTS_EVENT_SIZE);

RawSerial pc(USBTX,USBRX); //tx,rx
RawSerial uart(D1,D0); //tx,rx

RawSerial xbee(D12, D11);

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);

Ticker servo_ticker;
Ticker encoder_ticker;
Ticker encoder_ticker1;
PwmOut pin8(D5), pin9(D9);
DigitalInOut pin10(D10);
DigitalIn pin3(D3);
DigitalIn pin4(D4);
float object[2];
int ans = 0;
int init = 0;
char image = 'a';

void xbee_rx_interrupt(void);
void xbee_rx(void);
void reply_messange(char *xbee_reply, char *messange);
void check_addr(char *xbee_reply, char *messenger);

void getnumber(Arguments *in, Reply *out);

RPCFunction rpcnumber(&getnumber, "getnumber");

BBCar car(pin8, pin9, servo_ticker);

//void xbee_thread(){

//}

void go_call(){
        // please contruct you own calibration table with each servo
    pc.baud(9600);
    double pwm_table0[] = {-150, -120, -90, -60, -30, 0, 30, 60, 90, 120, 150};
    
    
    double speed_table0[] = {-16.606, -16.638, -15.840, -13.129, -6.908, 0.000, 5.732, 11.912, 15.621, 16.738, 17.216};
    
    double pwm_table1[] = {-150, -120, -90, -60, -30, 0, 30, 60, 90, 120, 150};
    double speed_table1[] = {-16.652, -16.366, -15.409, -12.139, -5.461, 0.000, 5.430, 12.229, 15.559, 16.675, 17.074};

    // first and fourth argument : length of table
    car.setCalibTable(11, pwm_table0, speed_table0, 11, pwm_table1, speed_table1);

    parallax_encoder encoder0(pin3, encoder_ticker);
    parallax_encoder encoder1(pin4, encoder_ticker1);
    parallax_ping  ping1(pin10);

    led1 = 1;
    led2 = 1;
    led3 = 1;
    //1  //16.7 [s]

    car.goStraightCalib(8);
    wait(15.7);
    
    car.stop();
             
    wait(1.0);

    //2  // right_turn
    
    
    encoder0.reset();
    car.turn(25,0.3);
    while(encoder0.get_cm()<19) wait_ms(50);
    car.stop();

    wait(1.0);

    led1 = 1;
    led2 = 0;
    led3 = 1;

    //3  /9.5[s]
    
    
    car.goStraightCalib(8);
    wait(8.5);
    
    car.stop();

    wait(1.0);

    
    //4
    //back parking
    
    encoder0.reset();
    car.turn(-25,0.3);
    while(encoder0.get_cm()<27) wait_ms(50);
    
    car.stop();

    wait(1.0);


    //5  //back_parking
    encoder0.reset();
    car.goStraightCalib(-4);

    wait(4.0);
    car.stop();
    wait(1.0);

    
    //6   [s]
    encoder0.reset();
    car.goStraightCalib(8);
    
    wait(1.0);
    car.stop();




    //snapshot  2s
    for(int k=0;k<5;k++) {
      
      if(k==0){
        char s[21];
        sprintf(s,"image_classification");
        uart.puts(s);
      }

      if(uart.readable()){
          image = uart.getc(); 
          break;     
      }
      wait(1.0);
    }



    //7
    car.goStraightCalib(8);
    
    wait(1.25);
    car.stop();
    wait(1.0);

    //8

    encoder1.reset();

    car.turn(25,-0.3);
    while(encoder1.get_cm()<20) wait_ms(50);
    car.stop();
    

    wait(1.0);

    
    //9  5[s]
    
   
    car.goStraightCalib(8);
    wait(3.15);
    
    car.stop();
    
    wait(1.0);

    led1 = 1;
    led2 = 1;
    led3 = 0;
    //10   left_turn : [s]

    encoder1.reset();

    car.turn(25,-0.3);
    while(encoder1.get_cm()<20) wait_ms(50);
    car.stop();
    

    wait(1.0);

    
    //11  //15.8[s]
    //middle running
    
    car.goStraightCalib(8);
    wait(13.5);
    car.stop();
             
    
    car.stop();

    wait(1.0);


    //12  left_turn : [s]
    encoder1.reset();

    car.turn(25,-0.3);
    while(encoder1.get_cm()<21) wait_ms(50);
    car.stop();
    

    wait(1.0);


    led1 = 0;
    led2 = 1;
    led3 = 1;

    //13  4[s]
    
    car.goStraightCalib(8);
    wait(4.25);
  
    car.stop();
    
    wait(1.0);

    //14  left_turn :  [s]
    //turn to mission2
    encoder1.reset();

    car.turn(25,-0.3);
    while(encoder1.get_cm()<19) wait_ms(50);
    car.stop();
    

    wait(1.0);


    /*
    //14  5.5[s]
    encoder0.reset();
    
    car.goStraightCalib(8);
    wait(3.5);
    
    car.stop();
    
    wait(1.0);*/

    /*
    //a
    encoder1.reset();
    car.turn(-25,0.3);
    while(encoder1.get_cm()<5) wait_ms(50);
    //wait(1.8);
    car.stop();

    wait(1.0);
    //b
    encoder1.reset();
    car.turn(25,0.3);
    while(encoder1.get_cm()<5) wait_ms(50);
    wait(2.0);
    car.stop();
    */

    //wait(1.0);


    //15
    object[0] = (float)ping1;
    wait(1.0);
    encoder1.reset();
    car.turn(15,-0.3);
    while(encoder1.get_cm()<1) wait_ms(50);
    object[1] = (float)ping1;
    //wait(2.0);
    car.stop();

    if(object[0]<70){
        if(object[1]>73)
          ans = 0;
        else{
          ans = 1;
        }
    }
    else{
        if(object[1]>object[0])
          ans = 2;
        else{
          ans = 3;
        }

    }

    wait(1.0);

    //16

    //left
    encoder1.reset();
    car.turn(25,-0.3);
    while(encoder1.get_cm()<18) wait_ms(50);
    car.stop();
    wait(1.0);

    encoder1.reset();
    car.turn(25,-0.3);
    while(encoder1.get_cm()<20) wait_ms(50);
    car.stop();
    wait(1.0);

    car.goStraightCalib(3);
    wait(0.75);
  
    car.stop();
    
    wait(1.0);

    encoder1.reset();
    car.turn(25,-0.3);
    while(encoder1.get_cm()<21) wait_ms(50);
    car.stop();
    wait(1.0);
    /*
    encoder1.reset();
    car.turn(25,-0.3);
    while(encoder1.get_cm()<17) wait_ms(50);
    wait(1.0);
    //d  right_turn_fast : [s]
    encoder1.reset();
    car.turn(25,-0.3);
    while(encoder1.get_cm()<20) wait_ms(50);
    //wait(2.0);
    car.stop();

    wait(1.0);
    */

    
    
    //17
    encoder0.reset();
    
    car.goStraightCalib(8);
    wait(5.0);
    car.stop();
            
    wait(1.0);


 /*   //16 left_turn : [s]
    encoder1.reset();

    car.turn(25,-0.3);
    while(encoder1.get_cm()<21) wait_ms(50);
    car.stop();
    

    wait(1.0);

    //17
    encoder0.reset();
    
    car.goStraightCalib(8);
    wait(5.0);
    
    car.stop();
    
    wait(1.0);
*/
    led1 = 0;
    led2 = 0;
    led3 = 0;

    //18 left_turn : [s]
    encoder1.reset();

    car.turn(25,-0.3);
    while(encoder1.get_cm()<20) wait_ms(50);
    car.stop();
    

    wait(1.0);


    //19  //16[s]
    encoder0.reset();
    
    car.goStraightCalib(8);
    wait(20.0);

    car.stop();

    wait(1.0);
}


/*void go_thread(){   
    queue1.call(go_call);  
}*/


int main() {
   uart.baud(9600);
   thread1.start(go_call);
   //sw2.rise(go_thread);
   char xbee_reply[4];

  // XBee setting
  
  xbee.baud(9600);
  xbee.printf("+++");
  xbee_reply[0] = xbee.getc();
  xbee_reply[1] = xbee.getc();
  if(xbee_reply[0] == 'O' && xbee_reply[1] == 'K'){
    pc.printf("enter AT mode.\r\n");
    xbee_reply[0] = '\0';
    xbee_reply[1] = '\0';
  }
  xbee.printf("ATMY 0x240\r\n");
  reply_messange(xbee_reply, "setting MY : 0x240");

  xbee.printf("ATDL 0x140\r\n");
  reply_messange(xbee_reply, "setting DL : 0x140");

  xbee.printf("ATID 0x4\r\n");
  reply_messange(xbee_reply, "setting PAN ID : 0x1");

  xbee.printf("ATWR\r\n");
  reply_messange(xbee_reply, "write config");

  xbee.printf("ATMY\r\n");
  check_addr(xbee_reply, "MY");

  xbee.printf("ATDL\r\n");
  check_addr(xbee_reply, "DL");

  xbee.printf("ATCN\r\n");
  reply_messange(xbee_reply, "exit AT mode");
  xbee.getc();

  // start
  pc.printf("start\r\n");
    thread2.start(callback(&queue, &EventQueue::dispatch_forever));

    xbee.attach(xbee_rx_interrupt, Serial::RxIrq);
}



void xbee_rx_interrupt(void)
{
  xbee.attach(NULL, Serial::RxIrq); // detach interrupt
  queue.call(&xbee_rx);
}

void xbee_rx(void)
{
  char buf[100] = {0};
  char outbuf[100] = {0};
  while(xbee.readable()){
    for (int i=0; ; i++) {
      char recv = xbee.getc();
      if (recv == '\r') {
        pc.printf("\r\n");
        break;
      }
      buf[i] = pc.putc(recv);
    }
    RPC::call(buf, outbuf);
    pc.printf("%s\r\n", outbuf);
    wait(0.1);
  }
  xbee.attach(xbee_rx_interrupt, Serial::RxIrq); // reattach interrupt
}

void reply_messange(char *xbee_reply, char *messange){
  xbee_reply[0] = xbee.getc();
  xbee_reply[1] = xbee.getc();
  xbee_reply[2] = xbee.getc();
  if(xbee_reply[1] == 'O' && xbee_reply[2] == 'K'){
    pc.printf("%s\r\n", messange);
    xbee_reply[0] = '\0';
    xbee_reply[1] = '\0';
    xbee_reply[2] = '\0';
  }
}

void check_addr(char *xbee_reply, char *messenger){
  xbee_reply[0] = xbee.getc();
  xbee_reply[1] = xbee.getc();
  xbee_reply[2] = xbee.getc();
  xbee_reply[3] = xbee.getc();
  pc.printf("%s = %c%c%c\r\n", messenger, xbee_reply[1], xbee_reply[2], xbee_reply[3]);
  xbee_reply[0] = '\0';
  xbee_reply[1] = '\0';
  xbee_reply[2] = '\0';
  xbee_reply[3] = '\0';
}

void getnumber(Arguments *in, Reply *out){
   init++;
   if(init<16){
     xbee.printf("go_straight\r\n");
   }
   else if(init<20){ 
     xbee.printf("turn_left\r\n");  
   }
   else if(init<27){
     xbee.printf("go_straight\r\n");
   }
   else if(init<39){
     xbee.printf("back_parking\r\n");
   }
   else if(init<53){
     xbee.printf("snapshot\r\n");
   }
   else if(init<57){
     xbee.printf("turn_right\r\n");
   }
   else if(init<60){
     xbee.printf("go_straight\r\n");
   }
   else if(init<63){
     xbee.printf("leave mission1\r\n");
   }
   else if(init<66){
     xbee.printf("image : %c\r\n",image);
   }
   else if(init<78){
     xbee.printf("go_straight\r\n");
   }
   else if(init<81){
     xbee.printf("turn_right\r\n");
   }
   else if(init<85){
     xbee.printf("go_straight\r\n");
   }
   else if(init<89){
     xbee.printf("turn_right\r\n");
   }
   else if(init<95){
     xbee.printf("object_classification\r\n");
   }
   else if(init<107){
     xbee.printf("turn_left\r\n");
   }
   else if(init<125){
     xbee.printf("leave mission2\r\n");
   }
   else if(init<128){ // xbee.print(object)
        xbee.printf("object : %d\r\n",ans);
   }
   else{
     ans = ans;
   }

}