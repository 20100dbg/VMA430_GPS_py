import serial
import time

class UBX_msg(object):
    def __init__(self):
        super(UBX_msg, self).__init__()
        self.class_byte = None
        self.id_byte = None
        self.payload_length = None
        self.msg = None
        self.CK_A = None
        self.CK_B = None

class Time_UTC(object):
    def __init__(self):
        super(Time_UTC, self).__init__()
        self.year = None
        self.month = None
        self.day = None
        self.hour = None
        self.minute = None
        self.second = None
        self.valid = None

class Location(object):
    def __init__(self):
        super(Location, self).__init__()
        self.longitude = None
        self.latitude = None
        self.valid = None


NAV_MODE = {'pedestrian': 0x03, 'automotive': 0x04, 'sea': 0x05, 'airborne': 0x06}
DATA_RATE = {'1HZ': 0xE803, '2HZ': 0xFA01, '3_33HZ': 0x2C01, '4HZ': 0xFA00}
PORT_RATE = {4800: 0xC01200, 9600: 0x802500, 19200: 0x004B00, 38400: 0x009600, 57600: 0x00E100, 115200: 0x00C200, 230400: 0x008400}
UBX_SYNC = {1: 0xB5, 2: 0x62}

# definition of UBX class IDs
# source: U-blox7 V14 Receiver Description Protocol page 88 https://www.u-blox.com/sites/default/files/products/documents/u-blox7-V14_ReceiverDescriptionProtocolSpec_%28GPS.G7-SW-12001%29_Public.pdf
NAV_CLASS = 0x01 # Navigation Results: Position, Speed, Time, Acc, Heading, DOP, SVs used
RXM_CLASS = 0x02 # Receiver Manager Messages: Satellite Status, RTC Status
INF_CLASS = 0x04 # Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
ACK_CLASS = 0x05 # Ack/Nack Messages: as replies to CFG Input Messages
CFG_CLASS = 0x06 # Configuration Input Messages: Set Dynamic Model, Set DOP Mask, Set Baud Rate, etc
MON_CLASS = 0x0A # Monitoring Messages: Comunication Status, CPU Load, Stack Usage, Task Status
AID_CLASS = 0x0B # AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
TIM_CLASS = 0x0D # Timing Messages: Time Pulse Output, Timemark Results
LOG_CLASS = 0x21 # Logging Messages: Log creation, deletion, info and retrieval


class VMA430(object):
    def __init__(self):
        super(VMA430, self).__init__()

        self.utc_time = Time_UTC()
        self.location = Location()
        self.latest_msg = UBX_msg()


    def begin(self, baudrate):

        port = '/dev/ttyS0'
        self.baudrate = baudrate
        self.nav_mode = NAV_MODE['pedestrian']
        self.data_rate = DATA_RATE['4HZ']
        self.serial = serial.Serial(port=port, baudrate=self.baudrate,
                    parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS, timeout=1)




    def generateConfiguration(self):
        arr = bytearray()
        arr.append(self.nav_mode)
        arr.append(self.data_rate) # arr[1] et arr[2]
        arr.append(self.baudrate >> 16)
        arr.append(self.baudrate >> 8)
        arr.append(self.baudrate & 0xFF)
        arr.append(int(self.GLLSentence))
        arr.append(int(self.GSVSentence))
        arr.append(int(self.RMCSentence))
        arr.append(int(self.VTGSentence))

        return arr

    def sendConfiguration():
        
        settings = generateConfiguration()
        gpsSetSuccess = 0
        print("Configuring u-Blox GPS initial state...");

        #Generate the configuration string for Navigation Mode
        setNav = [0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, *settings, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        calcChecksum(setNav[2:len(setNav)-4])

        #Generate the configuration string for Data Rate
        setDataRate = [0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, settings[1], settings[2], 0x01, 0x00, 0x01, 0x00, 0x00, 0x00]
        calcChecksum(setDataRate[2:len(setDataRate)-4])

        #Generate the configuration string for Baud Rate
        setPortRate = [0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, settings[3], settings[4], settings[5], 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        calcChecksum(setPortRate[2:len(setPortRate)-4])

        setGLL = [0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B]
        setGSA = [0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32]
        setGSV = [0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39]
        setRMC = [0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40]
        setVTG = [0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46]

        gpsStatus = [False, False, False, False, False, False, False]


        while gpsSetSuccess < 3:
        
            print("Setting Navigation mode...")

            sendUBX(setNav)     #Send UBX Packet
            gpsSetSuccess += getUBX_ACK(setNav[2]) #Passes Class ID and Message ID to the ACK Receive function

            if gpsSetSuccess == 5:
                gpsSetSuccess -= 4
                time.sleep(500)
                lowerPortRate = [0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA2, 0xB5]
                sendUBX(lowerPortRate)
                time.sleep(2000);

            if gpsSetSuccess == 6:
                gpsSetSuccess -= 4
            if gpsSetSuccess == 10:
                gpsStatus[0] = True;

        if gpsSetSuccess == 3:
            print("Navigation mode configuration failed.");
            
        gpsSetSuccess = 0;

        if settings[4] != 0x25:
            print("Setting Port Baud Rate... ");
            sendUBX(setPortRate);
            print("Success!");
            delay(500);


    def sendUBX(self, UBXmsg):        
        self.serial.write(UBXmsg)


    def setUBXNav(self):
        setNAVUBX = [0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x21, 0x01, 0x00, 0x00]
        setNAVUBX_pos = [0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x00, 0x00]
        print("Enabling UBX time NAV data");
        
        CK_A, CK_B = 0, 0

        for i in range(7):
            CK_A = CK_A + setNAVUBX[i + 2]
            CK_B = CK_B + CK_A

        setNAVUBX[9] = CK_A
        setNAVUBX[10] = CK_B

        sendUBX(setNAVUBX)
        getUBX_ACK(setNAVUBX[2]);

        print("Enabling UBX position NAV data");
        
        CK_A, CK_B = 0, 0
        for i in range(7):
            CK_A = CK_A + setNAVUBX_pos[i + 2]
            CK_B = CK_B + CK_A

        setNAVUBX_pos[9] = CK_A
        setNAVUBX_pos[10] = CK_B

        sendUBX(setNAVUBX_pos)
        getUBX_ACK(setNAVUBX_pos[2])

    def getconfig(self):
        get_cfg_message = [UBX_SYNC_1, UBX_SYNC_2, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00]

        CK_A, CK_B = 0, 0
        for i in range(4):
            CK_A = CK_A + get_cfg_message[i + 2]
            CK_B = CK_B + CK_A

        get_cfg_message[6] = CK_A
        get_cfg_message[7] = CK_B
        
        sendUBX(get_cfg_message)

    def getUBX_packet(self):
        sync_chars = [UBX_SYNC_1, UBX_SYNC_2]
        UBX_packet = [UBX_SYNC_1, UBX_SYNC_2, 0x00, 0x00]
    
        length_bytes = [0x00, 0x00]
        payload_length = 0
        
        i = 0
        CK_A = 0
        CK_B = 0
        checksum_idx = 0
        ubxWait = time.time()
        received_valid_ubx = False

        while True:

            if (time.time() - ubxWait > 1500):
                print("TimeOut UBX packet!")
                break

            if self.serial.available():
                incoming_char = self.serial.read_until(expected='')
                print(incoming_char)
                print(" ")

                if i < 2:
                    if incoming_char == sync_chars[i]:
                        i += 1

                elif i > 1:
                    print("i: ")
                    print(i)
                    
                    if i == 2:
                        print("got class byte")
                        class_byte_temp = incoming_char
                    elif i == 3:
                        print("got id byte")
                        id_byte_temp = incoming_char
                    elif i == 4:
                        print("got length byte 1")
                        length_bytes[0] = incoming_char
                    elif i == 5:
                        length_bytes[1] = incoming_char
                        payload_length = length_bytes[1] << 8 | length_bytes[0]

                        print("Expecting payload with length: ")
                        print(payload_length)

                    if i > 5 and checksum_idx == 0:

                        if payload_length > 40:
                            print("Invalid UBX packet length!");
                            break

                        if i - 5 <= payload_length:
                            buffer_msg[i - 6] = incoming_char
                        else:
                            print("Got start of checksum")
                            checksum_idx = i

                    if checksum_idx != 0:
                    
                        if i == checksum_idx:
                            CK_A = incoming_char

                        elif i == checksum_idx + 1:
                            CK_B = incoming_char
                            
                            print("Complete msg: ")
                            print(class_byte_temp, HEX)
                            print(" ")
                            print(id_byte_temp, HEX)
                            print(" Payload length:")
                            print(payload_length)
                            print(" Payload: ")
                            
                            for j in range(payload_length):
                                print(buffer_msg[j], HEX)
                                print(" ")

                            received_valid_ubx = True

                            self.latest_msg.class_byte = class_byte_temp
                            self.latest_msg.id_byte = id_byte_temp
                            self.latest_msg.payload_length = payload_length
                            self.latest_msg.msg = buffer_msg
                            self.latest_msg.CK_A = CK_A
                            self.latest_msg.CK_B = CK_B

                    i+= 1
        return received_valid_ubx


    def parse_ubx_data(self):
        if self.latest_msg.class_byte == NAV_CLASS:
            if self.latest_msg.id_byte == LOG_CLASS:
                self.parse_nav_timeutc()
            elif self.latest_msg.id_byte == RXM_CLASS:
                self.parse_nav_pos()



    def parse_nav_timeutc(self):

        if self.latest_msg.payload_length != 20:
            return False

        msg_data = self.latest_msg.msg
        self.utc_time.year = msg_data[12] | (msg_data[13]<<8)
        self.utc_time.month = msg_data[14]
        self.utc_time.day = msg_data[15]
        self.utc_time.hour = msg_data[16]
        self.utc_time.minute = msg_data[17]
        self.utc_time.second = msg_data[18]
        
        print("Validaty time data:", msg_data[19])
        self.utc_time.valid = (msg_data[19] == 0x07)
        return True


    def parse_nav_pos(self):
        temp_lon, temp_lat, temp_val = 0, 0, 0
        longitude, latitude = 0.0, 0.0

        #byte test_arr[] = {0x00, 0x45, 0x62, 0x1D, 0x4B, 0xE0, 0x4F, 0x02, 0x4A, 0x34, 0x65, 0x1E, 0x39, 0x5A, 0x00, 0x00, 0x63, 0xA6, 0xFF, 0xFF, 0xD7, 0x39, 0x01, 0x00, 0x9F, 0xB9, 0x00, 0x00};
        msg_data = self.latest_msg.msg

        if self.latest_msg.payload_length != 28:
            return False

        temp_lon = extractLong(4, msg_data);
        self.location.longitude = temp_lon*0.0000001
        
        temp_lat = extractLong(8, msg_data)
        self.location.latitude = temp_lat*0.0000001

        return True;


    def getUBX_ACK(msgID):
        CK_A, CK_B = 0, 0

        ackWait = millis()
        ackPacket = [0xB5, 0x62, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        i = 0

        while True:

            if self.serial.available():
                incoming_char = self.serial.read()

                if incoming_char == ackPacket[i]:
                    i += 1
                elif i > 2:
                    ackPacket[i] = incoming_char
                    i += 1

            if i > 9:
                break

            if (millis() - ackWait) > 1500:
                print("ACK Timeout")
                return 5

            if i == 4 and ackPacket[3] == 0x00:
                print("NAK Received")
                return 1

        #for (i = 2; i < 8; i++):
        for i in range(2,8):
            CK_A = CK_A + ackPacket[i]
            CK_B = CK_B + CK_A

        if msgID[0] == ackPacket[6] and msgID[1] == ackPacket[7] and CK_A == ackPacket[8] and CK_B == ackPacket[9]:
            print("Success! ACK Received! ")
            #printHex(ackPacket, sizeof(ackPacket))
            return 10

        else:
            print("ACK Checksum Failure: ")
            #printHex(ackPacket, sizeof(ackPacket))
            time.sleep(1000)
            return 1


    def extractLong(self, spotToStart, msg_data):
        val = 0;
        val |= msg_data[spotToStart + 0] << 8 * 0
        val |= msg_data[spotToStart + 1] << 8 * 1
        val |= msg_data[spotToStart + 2] << 8 * 2
        val |= msg_data[spotToStart + 3] << 8 * 3
        return val


    def calcChecksum(self, checksumPayload, payloadSize):
        CK_A, CK_B = 0, 0
        for i in range (payloadSize):
            CK_A = CK_A + checksumPayload[i]
            CK_B = CK_B + CK_A;

        checksumPayload[payloadSize] = CK_A;
        checksumPayload[payloadSize+1] = CK_B;


gps = VMA430()
gps.begin(9600)