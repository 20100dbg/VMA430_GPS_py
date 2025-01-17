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
UBX_SYNC = b'\xB5\x62'

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
        arr.append(self.baudrate.to_bytes(3, 'little'))
        arr.append(int(self.GLLSentence))
        arr.append(int(self.GSVSentence))
        arr.append(int(self.RMCSentence))
        arr.append(int(self.VTGSentence))

        return arr

    def sendConfiguration(self):
        
        settings = self.generateConfiguration()
        gpsSetSuccess = 0
        print("Configuring u-Blox GPS initial state...");

        #Generate the configuration string for Navigation Mode
        setNav = [0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF] + settings + [0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.calcChecksum(setNav[2:len(setNav)-4])

        #Generate the configuration string for Data Rate
        setDataRate = [0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, settings[1], settings[2], 0x01, 0x00, 0x01, 0x00, 0x00, 0x00]
        self.calcChecksum(setDataRate[2:len(setDataRate)-4])

        #Generate the configuration string for Baud Rate
        setPortRate = [0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, settings[3], settings[4], settings[5], 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.calcChecksum(setPortRate[2:len(setPortRate)-4])

        setGLL = [0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B]
        setGSA = [0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32]
        setGSV = [0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39]
        setRMC = [0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40]
        setVTG = [0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46]

        gpsStatus = [False, False, False, False, False, False, False]


        while gpsSetSuccess < 3:
        
            print("Setting Navigation mode...")

            self.sendUBX(setNav)     #Send UBX Packet
            gpsSetSuccess += self.getUBX_ACK(setNav[2:4]) #Passes Class ID and Message ID to the ACK Receive function

            if gpsSetSuccess == 5:
                gpsSetSuccess -= 4
                time.sleep(500)
                lowerPortRate = [0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA2, 0xB5]
                self.sendUBX(lowerPortRate)
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
            self.sendUBX(setPortRate);
            print("Success!");
            time.sleep(0.5);


    def sendUBX(self, UBXmsg):
        print(f"[+] SENDING {UBXmsg}")
        self.serial.write(UBXmsg)
        time.sleep(0.5)


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

        self.sendUBX(setNAVUBX)
        self.getUBX_ACK(setNAVUBX[2:4]);

        print("Enabling UBX position NAV data");
        
        CK_A, CK_B = 0, 0
        for i in range(7):
            CK_A = CK_A + setNAVUBX_pos[i + 2]
            CK_B = CK_B + CK_A

        setNAVUBX_pos[9] = CK_A
        setNAVUBX_pos[10] = CK_B

        self.sendUBX(setNAVUBX_pos)
        self.getUBX_ACK(setNAVUBX_pos[2:4])

    def getconfig(self):
        #get_cfg_message = [UBX_SYNC[0], UBX_SYNC[1], 0x06, 0x00, 0x00, 0x00, 0x00, 0x00]
        get_cfg_message = UBX_SYNC + b'\x06\x00\x00\x00\x00\x00'

        CK_A, CK_B = 0, 0
        for i in range(4):
            CK_A = CK_A + get_cfg_message[i + 2]
            CK_B = CK_B + CK_A

        get_cfg_message[6] = CK_A
        get_cfg_message[7] = CK_B
        
        self.sendUBX(get_cfg_message)

    def getUBX_packet(self):
        #UBX_packet = [UBX_SYNC[0], UBX_SYNC[1], 0x00, 0x00]
        UBX_packet = UBX_SYNC + b'\x00\x00'
    
        payload_length = 0
        
        i = 0
        CK_A = 0
        CK_B = 0
        checksum_idx = 0
        ubxWait = time.time()
        received_valid_ubx = False


        if (time.time() - ubxWait > 1.500):
            print("TimeOut UBX packet!")
            break

        data = self.serial.read_until(expected='')
        print(f"Received : {data}")

        if data[0:2] != UBX_SYNC:
            print("[-] Error !")

        class_byte_temp = data[2]
        id_byte_temp = data[3]
        #length_bytes = data[4:5]
        payload_length = int.from_bytes(data[4:5], byteorder='little')

        print(f"Expecting payload with length: {payload_length}")

        payload = data[5:payload_length+5]
        checksum = data[payload_length+5:]

        print(f"Received {self.btohex(payload)}")

        if payload_length > 40:
            print("Invalid UBX packet length!");


        checksum_idx = payload_length+5
        CK_A = data[checksum_idx]
        CK_B = data[checksum_idx + 1]

        #TODO : pas de vérification du checksum ?
        received_valid_ubx = True

        self.latest_msg.class_byte = class_byte_temp
        self.latest_msg.id_byte = id_byte_temp
        self.latest_msg.payload_length = payload_length
        self.latest_msg.msg = buffer_msg
        self.latest_msg.CK_A = CK_A
        self.latest_msg.CK_B = CK_B

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
        self.utc_time.year = int.from_bytes(msg_data[12:13], byteorder='little')
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

        temp_lon = self.extractLong(4, msg_data);
        self.location.longitude = temp_lon*0.0000001
        
        temp_lat = self.extractLong(8, msg_data)
        self.location.latitude = temp_lat*0.0000001

        return True;


    def btohex(self, b):
        """ Get bytes to proper hex notation """
        return ' '.join(['{:02X}'.format(x) for x in b])


    def getUBX_ACK(self, msgID):
        CK_A, CK_B = 0, 0

        ackWait = time.time()
        ackPacket = b'\xB5\x62\x05'
        i = 0

        data = self.serial.read_until(expected='')
        if not data:
            time.sleep(0.5)
            data = self.serial.read_until(expected='')
            if not data:
                print("ACK Timeout")
                return 5

        #print(f"[+] READ {self.btohex(data)}")
        #print(f"[+] READ {data}")

        if data[0:3] == ackPacket:
            print("Received ACK")
        elif data[3] == b'\x00':
            print("NAK Received")
            return 1

        print(f"data : {self.btohex(data[0:15])}...")
        
        ###################
        #zone à corriger
        checksum = data[2:8]

        #for (i = 2; i < 8; i++):
        for i in checksum:
            CK_A = CK_A + i
            CK_B = CK_B + CK_A

        print(f"checksum : {self.btohex(checksum)}")
        print(msgID, CK_A, CK_B)

        #quels indices dans msgID et checksum ?
        ####
        if msgID[0] == checksum[3] and msgID[1] == checksum[4] and CK_A == checksum[5] and CK_B == checksum[6]:
            print("Success! ACK Received! ")
            return 10

        else:
            print("ACK Checksum Failure: ")
            return 1


    def extractLong(self, spotToStart, msg_data):
        val = 0;
        val |= msg_data[spotToStart + 0] << 8 * 0
        val |= msg_data[spotToStart + 1] << 8 * 1
        val |= msg_data[spotToStart + 2] << 8 * 2
        val |= msg_data[spotToStart + 3] << 8 * 3

        return int.from_bytes(msgdata[spotToStart:spotToStart+4], byteorder='little')
        #return val


    def calcChecksum(self, checksumPayload):
        CK_A, CK_B = 0, 0
        for i in range (len(checksumPayload)):
            CK_A = CK_A + checksumPayload[i]
            CK_B = CK_B + CK_A;

        checksumPayload.append(CK_A)
        checksumPayload.append(CK_B)


gps = VMA430()
gps.begin(9600)
gps.setUBXNav()

while True:

    print(f"got packet : {gps.getUBX_packet()}")
    print(f"is UTC valid : {gps.utc_time.valid}")
    print(f"longitude : {gps.location.longitude}")
    print(f"latitude : {gps.location.latitude}")


    time.sleep(1)