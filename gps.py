import serial
import time
import struct

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

    def to_string(self):
        return f"{self.year}-{self.month}-{self.day} {self.hour}:{self.minute}:{self.second} ({self.valid})"

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
MON_CLASS = 0x0A #10 Monitoring Messages: Comunication Status, CPU Load, Stack Usage, Task Status
AID_CLASS = 0x0B #11 AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
TIM_CLASS = 0x0D #13 Timing Messages: Time Pulse Output, Timemark Results
LOG_CLASS = 0x21 #33 Logging Messages: Log creation, deletion, info and retrieval


class VMA430(object):
    def __init__(self):
        super(VMA430, self).__init__()

        self.utc_time = Time_UTC()
        self.location = Location()


    def begin(self, baudrate):

        port = '/dev/ttyS0'
        self.baudrate = baudrate
        self.nav_mode = NAV_MODE['pedestrian']
        self.data_rate = DATA_RATE['4HZ']
        self.serial = serial.Serial(port=port, baudrate=self.baudrate,
                    parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS, timeout=1)



    def generateConfiguration(self):
        
        tmp_data_rate = self.data_rate.to_bytes(2, 'little')
        tmp_baudrate = self.baudrate.to_bytes(3, 'little')
        print(f"tmp_baudrate {tmp_baudrate}")

        arr = bytearray()
        arr.append(self.nav_mode)
        arr.append(tmp_data_rate[0])
        arr.append(tmp_data_rate[1])
        arr.append(tmp_baudrate[0])
        arr.append(tmp_baudrate[1])
        arr.append(tmp_baudrate[2])
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
                time.sleep(0.1)
                lowerPortRate = [0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA2, 0xB5]
                self.sendUBX(lowerPortRate)
                time.sleep(0.1);

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
            time.sleep(0.1);


    def sendUBX(self, UBXmsg):
        print(f"[+] SENDING {UBXmsg}")
        self.serial.write(UBXmsg)
        time.sleep(0.1)


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
        get_cfg_message = UBX_SYNC + b'\x06\x00\x00\x00\x00\x00'

        CK_A, CK_B = 0, 0
        for i in range(4):
            CK_A = CK_A + get_cfg_message[i + 2]
            CK_B = CK_B + CK_A

        get_cfg_message[6] = CK_A
        get_cfg_message[7] = CK_B
        
        self.sendUBX(get_cfg_message)

    def getUBX_packet(self):
        UBX_packet = UBX_SYNC + b'\x00\x00'
        received_valid_ubx = False
        receivedSync = False

        data = self.serial.read_until(expected='')
        #print(f"getUBX_packet : {self.btohex(data)}")
        """
        if not receivedSync:
            print("did not Received SYNC")
            return 1
        """

        for idx in range(0,len(data)-2,2):

            if data[idx:idx+2] == UBX_SYNC:
                #print("Received SYNC")
                receivedSync = True
                subpacket = data[idx:]
                #break
        
                class_byte = subpacket[2]

                # in [NAV_CLASS, RXM_CLASS, INF_CLASS, ACK_CLASS, CFG_CLASS, MON_CLASS, AID_CLASS, TIM_CLASS, LOG_CLASS]:
                if class_byte == NAV_CLASS:

                    id_byte = subpacket[3]
                    payload_length = int.from_bytes(subpacket[4:5], byteorder='little')

                    if payload_length == 0:
                        continue

                    payload = subpacket[5:payload_length+5]
                    checksum = subpacket[payload_length+5:]

                    """
                    print(f"class_byte : {class_byte}")
                    print(f"id_byte : {id_byte}")
                    print(f"payload length : {payload_length}")
                    print(f"payload : {self.btohex(payload)}")
                    print(f"checksum : {self.btohex(checksum[:15])}...")
                    """

                    if payload_length > 40:
                        print("Invalid UBX packet length!");

                    checksum_idx = payload_length+5
                    CK_A = subpacket[checksum_idx]
                    CK_B = subpacket[checksum_idx + 1]


                    if id_byte == LOG_CLASS:
                        self.parse_nav_timeutc(payload)
                    elif id_byte == RXM_CLASS:
                        self.parse_nav_pos(payload)

        if not receivedSync:
            print("[-] Did not received SYNC")
            return False

        return True



    def parse_nav_timeutc(self, payload):

        if len(payload) != 20:
            return False

        self.utc_time.year = int.from_bytes(payload[13:15], byteorder='little')
        self.utc_time.month = payload[15]
        self.utc_time.day = payload[16]
        self.utc_time.hour = payload[17]
        self.utc_time.minute = payload[18]
        self.utc_time.second = payload[19]
        
        print(f"date : {self.utc_time.to_string()}")
        self.utc_time.valid = (payload[14] == 0x07)
        return True


    def parse_nav_pos(self, payload):
        temp_lon, temp_lat, temp_val = 0, 0, 0
        longitude, latitude = 0.0, 0.0

        #byte test_arr[] = {0x00, 0x45, 0x62, 0x1D, 0x4B, 0xE0, 0x4F, 0x02, 0x4A, 0x34, 0x65, 0x1E, 0x39, 0x5A, 0x00, 0x00, 0x63, 0xA6, 0xFF, 0xFF, 0xD7, 0x39, 0x01, 0x00, 0x9F, 0xB9, 0x00, 0x00};

        """
        print(f"payload length : {len(payload)}")
        print(f"payload : {self.btohex(payload)}")
        """

        if len(payload) != 28:
            return False

        """
        print(f"temp_lon bytes : {self.btohex(payload[4:8])}")
        print(f"temp_lat bytes : {self.btohex(payload[8:12])}")

        temp_lon = int.from_bytes(payload[4:9], byteorder='little')
        self.location.longitude = temp_lon*0.0000001
        print(f"temp_lon {temp_lon}")
        """

        print(f"payload : {self.btohex(payload)}")
        
        print(f"test0 : {self.btohex(payload[4:8])}")
        test = self.extractLong(4, payload)
        print(f"test1 : {test}")
        test = struct.unpack('d', b'\x00\x00\x00\x00' + payload[4:8])[0]
        print(f"test2 : {test}")

        print(f"__________")

        print(f"test0 : {self.btohex(payload[8:12])}")
        test = self.extractLong(8, payload)
        print(f"test1 : {test}")
        test = struct.unpack('d', b'\x00\x00\x00\x00' + payload[8:12])[0]
        print(f"test2 : {test}")


        #temp_lat = int.from_bytes(payload[8:12], byteorder='little')
        #self.location.latitude = temp_lat*0.0000001
        #print(f"temp_lat {temp_lat}")

        return True;


    def btohex(self, b):
        """ Get bytes to proper hex notation """
        return ' '.join(['{:02X}'.format(x) for x in b])


    def getUBX_ACK(self, msgID):
        CK_A, CK_B = 0, 0

        ackWait = time.time()
        ackPacket = b'\xB5\x62\x05'
        receivedACK = False

        for x in range(5):
            data = self.serial.read_until(expected='')

            if data[0:3] == ackPacket:
                #print("[+] Received ACK")
                receivedACK = True
                break

        if not receivedACK:
            print("[-] Did not received ACK")
            return 1

        checksum = data[3:10]

        for i in range(2,8):
            CK_A = CK_A + data[i]
            CK_B = CK_B + CK_A

        if msgID[0] == checksum[3] and msgID[1] == checksum[4] and CK_A == checksum[5] and CK_B == checksum[6]:
            print("[+] ACK Received! ")
            return 10

        else:
            print("[-] ACK Checksum Failure: ")
            return 1


    def calcChecksum(self, checksumPayload):
        CK_A, CK_B = 0, 0
        for i in range (len(checksumPayload)):
            CK_A = CK_A + checksumPayload[i]
            CK_B = CK_B + CK_A;

        checksumPayload.append(CK_A)
        checksumPayload.append(CK_B)

    def extractLong(self, spotToStart, payload):
        val = 0;
        val |= payload[spotToStart + 0] << 8 * 0
        val |= payload[spotToStart + 1] << 8 * 1
        val |= payload[spotToStart + 2] << 8 * 2
        val |= payload[spotToStart + 3] << 8 * 3
        return val


gps = VMA430()
gps.begin(9600)
#gps.sendConfiguration()

gps.setUBXNav()

while True:
    print("---- MAIN LOOP ----")

    gps.getUBX_packet()

    """
    print(f"UTC : {gps.utc_time.to_string()}")
    print(f"longitude : {gps.location.longitude}")
    print(f"latitude : {gps.location.latitude}")
    """

    time.sleep(1)