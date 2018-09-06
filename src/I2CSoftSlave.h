
// Used by some platforms to put functions in ram.
#ifndef ICACHE_RAM_ATTR
#define ICACHE_RAM_ATTR
#endif

#define MAX_PAYLOAD_SIZE 5

class i2c_soft_slave {
  public:

    static int slave_sda;
    static int slave_scl;

    enum i2c_state {
      STOP = 0,
      START = 1,
      ADDR_0 = 2,
      ADDR_1 = 3,
      ADDR_2 = 4,
      ADDR_3 = 5,
      ADDR_4 = 6,
      ADDR_5 = 7,
      ADDR_6 = 8,
      RW_BIT = 9,
      ADDR_ACK = 10,
      DATA_0 = 11,
      DATA_1 = 12,
      DATA_2 = 13,
      DATA_3 = 14,
      DATA_4 = 15,
      DATA_5 = 16,
      DATA_6 = 17,
      DATA_7 = 18,
      DATA_ACK = 19
    };

    volatile static i2c_state state;

    struct i2c_packet {
      byte addr;
      bool w;
      byte dc;
      byte data[MAX_PAYLOAD_SIZE];
    };
    volatile static i2c_packet pack;
    static void (*on_read)(i2c_packet *pack);
    static void (*on_stop)(i2c_packet *pack);

    static void ICACHE_RAM_ATTR inline setInput(int input) {
      if (input) {
        pinMode(slave_sda, INPUT_PULLUP);
      } else {
        pinMode(slave_sda, OUTPUT);
      }
    }

    static void ICACHE_RAM_ATTR inline stop() {
      if (state != STOP) {
        //setInput(true);
        state = STOP;
        on_stop((i2c_packet *)&pack);
        memset((void *)&pack, 0, sizeof(i2c_packet));
      }
    }

    static void ICACHE_RAM_ATTR inline clock_rise() {
      if (state == STOP)
        return;

      // If no activa packet, eller to many bytes, just make sure we are in stopped state.
      if (pack.dc >= MAX_PAYLOAD_SIZE) {
        stop();
        return;
      }

      switch (state) {
        // First 7 bits is the address
        case STOP:
        case START:
          break;
        case ADDR_0:
        case ADDR_1:
        case ADDR_2:
        case ADDR_3:
        case ADDR_4:
        case ADDR_5:
        case ADDR_6:
          pack.addr <<= 1;
          pack.addr |= digitalRead(slave_sda);
          break;

        // Read rw-bit
        case RW_BIT:
          pack.w = !digitalRead(slave_sda);
          break;
        case ADDR_ACK:
          break;

        // If in read mode, read 8 bytes
        case DATA_0:
        case DATA_1:
        case DATA_2:
        case DATA_3:
        case DATA_4:
        case DATA_5:
        case DATA_6:
        case DATA_7:
          if (pack.w) {
            pack.data[pack.dc] <<= 1;
            pack.data[pack.dc] |= digitalRead(slave_sda);
          }
          break;
        case DATA_ACK:
          break;
      }

    }

    static void ICACHE_RAM_ATTR inline clock_fall() {
      if (state == STOP)
        return;


      switch (state) {
        // On r/w bit fall, pull data low as next one is an ACK
        case STOP:
        case START:
          break;
        case ADDR_0:
        case ADDR_1:
        case ADDR_2:
        case ADDR_3:
        case ADDR_4:
        case ADDR_5:
        case ADDR_6:
          break;

        case RW_BIT:
          setInput(false);
          digitalWrite(slave_sda, 0);
          break;

        // On falling clock after sending ACK bit, just release data bus.
        case ADDR_ACK:
          if (pack.w) {
            // WRITE FROM MASTER
            setInput(true);

          } else {

            // READ FROM MASTER
            setInput(false);
            on_read((i2c_packet *)&pack); // Let callback fill the data...
            digitalWrite(slave_sda, (pack.data[pack.dc] & 0x80) != 0); // Write first byte
          }
          break;

        // If in write mode, write 7 more bits
        case DATA_0:
          if (!pack.w) digitalWrite(slave_sda, (pack.data[pack.dc] & 0x40) != 0); break;
        case DATA_1:
          if (!pack.w) digitalWrite(slave_sda, (pack.data[pack.dc] & 0x20) != 0); break;
        case DATA_2:
          if (!pack.w) digitalWrite(slave_sda, (pack.data[pack.dc] & 0x10) != 0); break;
        case DATA_3:
          if (!pack.w) digitalWrite(slave_sda, (pack.data[pack.dc] & 0x8) != 0); break;
        case DATA_4:
          if (!pack.w) digitalWrite(slave_sda, (pack.data[pack.dc] & 0x4) != 0); break;
        case DATA_5:
          if (!pack.w) digitalWrite(slave_sda, (pack.data[pack.dc] & 0x2) != 0); break;
        case DATA_6:
          if (!pack.w) digitalWrite(slave_sda, (pack.data[pack.dc] & 0x1) != 0); break;

        // When DATA_7 Falls, we are done, and prepares for ack
        case DATA_7:
          if (pack.w) {
            // WRITE FROM MASTER
            setInput(false);
            digitalWrite(slave_sda, 0);
          } else {
            // READ FROM MASTER
            setInput(true);
          }
          pack.dc++;
          break;


        // On ack fall, prepare for send or read
        case DATA_ACK:

          if (pack.w) {
            // WRITE FROM MASTER
            setInput(true);
            if (digitalRead(slave_sda)) {
              stop();
              break;
            }

          } else {
            // READ FROM MASTER
            if (digitalRead(slave_sda)) {
              stop();
              break;
            //  return;
            }
            setInput(false);
            digitalWrite(slave_sda, pack.data[pack.dc] & 0x80); // Write the first bit of next byte
            // Writing to device.
          }
          break;
      }

      // If last data bit, the next bit will be handled the same as an ACK.
      if (state == DATA_ACK) {
        state = DATA_0;
      } else if (state != STOP) {
        int x = (int)state;
        x++;
        state = (i2c_state)x;
      }

    }


    static void print_packet(int count, i2c_packet *print_pack) {
      switch(print_pack->dc) {
        case 0:
          printf("count: %04d %s addr: 0x%02x data[%d]\n", count, print_pack->w ? "WRITE" : "READ ",
             print_pack->addr, print_pack->dc);
          break;
        case 1:
          printf("count: %04d %s addr: 0x%02x data[%d]: 0x%02x\n", count, print_pack->w ? "WRITE" : "READ ",
             print_pack->addr, print_pack->dc, print_pack->data[0]);
          break;
        case 2:
          printf("count: %04d %s addr: 0x%02x data[%d]: 0x%02x 0x%02x\n", count, print_pack->w ? "WRITE" : "READ ",
             print_pack->addr, print_pack->dc, print_pack->data[0], print_pack->data[1]);
          break;
        case 3:
          printf("count: %04d %s addr: 0x%02x data[%d]: 0x%02x 0x%02x 0x%02x \n", count, print_pack->w ? "WRITE" : "READ ",
             print_pack->addr, print_pack->dc, print_pack->data[0], print_pack->data[1], print_pack->data[2]);
          break;
        case 4:
          printf("count: %04d %s addr: 0x%02x data[%d]: 0x%02x 0x%02x 0x%02x 0x%02x\n", count, print_pack->w ? "WRITE" : "READ ",
             print_pack->addr, print_pack->dc, print_pack->data[0], print_pack->data[1], print_pack->data[2], print_pack->data[3]);
          break;
      }
    }



//#define DEBUG_STATE 1
    static void inline ICACHE_RAM_ATTR data_change() {
#ifdef DEBUG_TIMING
        digitalWrite(2, 1);
#endif
        bool sck = digitalRead(slave_scl);
        if (digitalRead(slave_sda)) {
           // If data rise during CLCK high, its always stop condition.
          if (sck == 1) {
            stop();
          }
        }
        //else {
        if (!digitalRead(slave_sda)) {
          if (sck == 1) {
            stop();

            // RESET counter
            state = START;
          }
        }

#ifdef DEBUG_TIMING
        digitalWrite(2, 0);
        digitalWrite(14, digitalRead(slave_sda) != 0);
#endif
#ifdef DEBUG_STATE
        digitalWrite(14, (state & 0b1) != 0);
        digitalWrite(16, (state & 0b10) != 0);
        digitalWrite(2, (state & 0b100) != 0);
        digitalWrite(0, (state & 0b1000) != 0);
        digitalWrite(10, (state & 0b10000) != 0);
#endif

    }

    static void inline ICACHE_RAM_ATTR clk_change() {
#ifdef DEBUG_TIMING
        digitalWrite(0, 1);
#endif
        if (digitalRead(slave_scl)) {
          i2c_soft_slave::clock_rise();
        } else {
          i2c_soft_slave::clock_fall();
          }
#ifdef DEBUG_TIMING
        digitalWrite(0, 0);
        digitalWrite(16, digitalRead(slave_scl) != 0);
#endif

#ifdef DEBUG_STATE
        digitalWrite(14, (state & 0b1) != 0);
        digitalWrite(16, (state & 0b10) != 0);
        digitalWrite(2, (state & 0b100) != 0);
        digitalWrite(0, (state & 0b1000) != 0);
        digitalWrite(10, (state & 0b10000) != 0);
#endif
    }

    i2c_soft_slave(int slave_sda_, int slave_scl_, void (*on_read_)(i2c_packet *pack), void (*on_stop_)(i2c_packet *pack)) {
      slave_sda = slave_sda_;
      slave_scl = slave_scl_;
      on_read = on_read_;
      on_stop = on_stop_;
      pinMode(slave_sda, INPUT_PULLUP);
      pinMode(slave_scl, INPUT_PULLUP);

#if defined(DEBUG_STATE) || defined(DEBUG_TIMING)
      pinMode(14, OUTPUT);
      pinMode(16, OUTPUT);
      pinMode(2, OUTPUT);
      pinMode(0, OUTPUT);
      pinMode(10, OUTPUT);
#endif
       setInput(true);
      state = STOP;

      attachInterrupt(digitalPinToInterrupt(slave_sda), data_change, CHANGE);
      attachInterrupt(digitalPinToInterrupt(slave_scl), clk_change, CHANGE);
    }
};
int i2c_soft_slave::slave_sda;
int i2c_soft_slave::slave_scl;
volatile i2c_soft_slave::i2c_state i2c_soft_slave::state;
volatile i2c_soft_slave::i2c_packet i2c_soft_slave::pack;
void (*i2c_soft_slave::on_read)(i2c_soft_slave::i2c_packet *pack);
void (*i2c_soft_slave::on_stop)(i2c_soft_slave::i2c_packet *pack);

