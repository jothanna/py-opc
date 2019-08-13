from .exceptions import FirmwareVersionError, SpiConnectionError
from .decorators import requires_firmware
from .lookup_table import OPC_LOOKUP

from time import sleep
import struct
import warnings
import re
import logging

import numpy

from .exceptions import firmware_error_msg

# set up a default logger
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

__version__ = "1.6.0"

__all__ = ['OPCN3', 'OPCN2', 'OPCN1']

class _OPC(object):
    """Generic class for any Alphasense OPC. Provides the common methods and calculations for each OPC. This class is designed to be the base class, and should not be used alone unless during development.

    :param spi_connection: spidev.SpiDev or usbiss.spi.SPI connection
    :param debug: Set true to print data to console while running
    :param model: Model number of the OPC ('N1' or 'N2' or 'N3') set by the parent class
    :param firmware: You can manually set the firmware version as a tuple. Ex. (18,2)
    :param max_cnxn_retries: Maximum number of times a connection will try to be made.
    :param retry_interval_ms: The sleep interval for the device between retrying to connect to the OPC. Units are in ms.

    :raises: opc.exceptions.SpiConnectionError

    :type spi_connection: spidev.SpiDev or usbiss.spi.SPI
    :type debug: boolean
    :type model: string
    :type max_cnxn_retries: int
    :type retry_interval_ms: int

    :rtype: opc._OPC

    """
    def __init__(self, spi_connection, firmware=None, max_cnxn_retries=5, retry_interval_ms=1000, **kwargs):
        self.cnxn       = spi_connection
        self.debug      = kwargs.get('debug', False)
        self.model      = kwargs.get('model', 'N2')

        if firmware is not None:
            major, minor = firmware[0], firmware[1]
            version = float("{}.{}".format(major, minor))
        else:
            major, minor, version = None, None, None

        self.firmware   = {'major': major, 'minor': minor, 'version': version}

        # Check to make sure the connection has the xfer attribute
        msg = ("The SPI connection must be a valid SPI master with "
               "transfer function 'xfer'")
        assert hasattr(spi_connection, 'xfer'), msg
        assert self.cnxn.mode == 1, "SPI mode must be 1"

        # Set the firmware version upon initialization IFF it hasn't been set manually
        i = 0

        self.firmware['version'] = 18.

        if self.firmware['version'] is None:
            while self.firmware['version'] is None:
                if i > max_cnxn_retries:
                    msg =   """
                            Your firmware version could not be automatically detected. This is usually caused by bad wiring or a poor power supply. If niether of these are likely candidates, please open an issue on the GitHub repository at https://github.com/dhhagan/py-opc/issues/new. Another option would be to
                            increase the max_cnxn_retries variable if you feel the serial communication is being held up for some reason.
                            """

                   ## raise FirmwareVersionError(msg)

                # store the info_string
                infostring = self.read_info_string()

                print infostring

                try:
                    self.firmware['version'] = int(re.findall("\d{3}", infostring)[-1])
                except Exception as e:
                  ##  logger.error("Could not parse the fimrware version from {}".format(infostring), exc_info=True)

                    # sleep for a period of time
                    sleep(retry_interval_ms / 1000)

                i += 1

        # At this point, we have a firmware version

        # If firmware version is >= 18, set the major and minor versions..
  #      try:
  #          if self.firmware['version'] >= 18.:
  #              self.read_firmware()
  #          else:
  #              self.firmware['major'] = self.firmware['version']
  #      except:
  #          logger.info("No firmware version could be read.")

        # We requested to wait until the device is connected
        if kwargs.get('wait', False) is not False:
            self.wait(**kwargs)

        else: # Sleep for a bit to alleviate issues
            sleep(1)

    def _16bit_unsigned(self, LSB, MSB):
        """Returns the combined LSB and MSB
        :param LSB: Least Significant Byte
        :param MSB: Most Significant Byte
        :type LSB: byte
        :type MSB: byte
        :rtype: 16-bit unsigned int
        """
        return (MSB << 8) | LSB

    def _calculate_float(self, byte_array):
        """Returns an IEEE 754 float from an array of 4 bytes
        :param byte_array: Expects an array of 4 bytes
        :type byte_array: array
        :rtype: float
        """
        #print "len(byte_array):",len(byte_array)
        if len(byte_array) != 4:
            return None
        return struct.unpack('f', struct.pack('4B', *byte_array))[0]

    def _calculate_mtof(self, mtof):
        """Returns the average amount of time that particles in a bin
        took to cross the path of the laser [units -> microseconds]
        :param mtof: mass time-of-flight
        :type mtof: float
        :rtype: float
        """
        return mtof / 3.0

    def _calculate_temp(self, vals):
        """Calculates the temperature in degrees celcius
        :param vals: array of bytes
        :type vals: array
        :rtype: float
        """
        print "temp: ", vals
        if len(vals) < 4:
            return None
        return ((vals[3] << 24) | (vals[2] << 16) | (vals[1] << 8) | vals[0]) 

    def _calculate_temp_uint(self, LSB, MSB):
        """Calculates the temperature in degrees celcius"""
        tempraw = ((MSB << 8) | LSB)
        temp = -45.0 + 175.0 * tempraw /(2**16-1)
        #print "temp raw/deg =", tempraw, temp
        return temp

    def _calculate_hum(self, vals):
        """Calculates the relative humidity in percent
        :param vals: array of bytes
        :type vals: array
        :rtype: float
        """
        if len(vals) < 4:
            return None
        return ((vals[3] << 24) | (vals[2] << 16) | (vals[1] << 8) | vals[0]) 

    def _calculate_hum_uint(self, LSB, MSB):
        hum = ((MSB << 8) | LSB)
        hum = 100.0 * hum / (2**16-1)
        return hum

    def _calculate_flowrate(self, LSB, MSB):
        """Calculates the temperature in degrees celcius
        :param vals: array of bytes
        :type vals: array
        :rtype: float
        """
        return ((MSB << 8) | LSB) / 100.0

    def _calculate_pressure(self, vals):
        """Calculates the pressure in pascals
        :param vals: array of bytes
        :type vals: array
        :rtype: float
        """
        if len(vals) < 4:
            return None

        return ((vals[3] << 24) | (vals[2] << 16) | (vals[1] << 8) | vals[0])

    def _calculate_period(self, vals):
        ''' calculate the sampling period in seconds '''
        if len(vals) < 4:
            return None
        if self.firmware < 16:
            return ((vals[3] << 24) | (vals[2] << 16) | (vals[1] << 8) | vals[0]) / 12e6
        else:
            return self._calculate_float(vals)

    def _calculate_period_uint(self, LSB, MSB):
        ''' calculate the sampling period in seconds '''
        return ((MSB << 8) | LSB) / 100.0


    def _MODBUS_CalcCRC(self, data, nbrOfBytes):

        POLYNOMIAL_MODBUS = 0xA001 #Generator polynomial for MODBUS crc
        InitCRCval_MODBUS = 0xFFFF #Initial CRC value

        crc = InitCRCval_MODBUS
        byteCtr = 0
        while(byteCtr<nbrOfBytes):
            crc ^= data[byteCtr]
            byteCtr+=1
            for i in range(0,8):
                if crc & 1:
                    crc >>=1
                    crc ^= POLYNOMIAL_MODBUS
                else:
                    crc>>=1
        return crc

    def wait(self, **kwargs):
        """Wait for the OPC to prepare itself for data transmission. On some devides this can take a few seconds
        :rtype: self
        :Example:
        >> alpha = opc.OPCN2(spi, debug=True).wait(check=200)
        >> alpha = opc.OPCN2(spi, debug=True, wait=True, check=200)
        """

        if not callable(self.on):
            raise UserWarning('Your device does not support the self.on function, try without wait')

        if not callable(self.histogram):
            raise UserWarning('Your device does not support the self.histogram function, try without wait')

        self.on()
        while True:
            try:
                if self.histogram() is None:
                    raise UserWarning('Could not load histogram, perhaps the device is not yet connected')

            except UserWarning as e:
                sleep(kwargs.get('check', 200) / 1000.)

        return self

    def lookup_bin_boundary(self, adc_value):
        """Looks up the bin boundary value in microns based on the lookup table provided by Alphasense.
            :param adc_value: ADC Value (0 - 4095)
            :type adc_value: int
            :rtype: float
        """
        if adc_value < 0:
            adc_value = 0

        if adc_value > 4095:
            adc_value = 4095

        return OPC_LOOKUP[adc_value]

    def calculate_bin_boundary(self, bb):
        """Calculate the adc value that corresponds to a specific bin boundary diameter in microns.
            :param bb: Bin Boundary in microns
            :type bb: float
            :rtype: int
        """

        return min(enumerate(OPC_LOOKUP), key = lambda x: abs(x[1] - bb))[0]

    def read_info_string(self):
        """Reads the information string for the OPC
        :rtype: string
        :Example:
        >>> alpha.read_info_string()
        'OPC-N2 FirmwareVer=OPC-018.2....................BD'
        """
        infostring = []

        # Send the command byte and sleep for 9 ms
        self.cnxn.xfer([0x3F])
        sleep(10e-3)

        # Read the info string by sending 60 empty bytes
        for i in range(60):
            # resp = self.cnxn.xfer([0x00])[0]
            resp = self.cnxn.xfer([0x3F])[0]
            infostring.append(chr(resp))

        sleep(0.1)

        return ''.join(infostring)

    def ping(self):
        """Checks the connection between the Raspberry Pi and the OPC
        :rtype: Boolean
        """
        b = self.cnxn.xfer([0xCF])[0]           # send the command byte
        sleep(0.1)
        return True if b == 0xF3 else False

    def __repr__(self):
        return "Alphasense OPC-{}v{}".format(self.model, self.firmware['version'])

###################################################################################
class OPCN3(_OPC):
    """Create an instance of the Alphasene OPC-N3. Currently supported by firmware
    versions 14-18. opc.OPCN3 inherits from the opc.OPC parent class.
    :param spi_connection: The spidev instance for the SPI connection.
    :type spi_connection: spidev.SpiDev
    :rtype: opc.OPCN3
    :raises: opc.exceptions.FirmwareVersionError
    :Example:
    >>> alpha = opc.OPCN3(spi)
    >>> alpha
    Alphasense OPC-N3v18.2
    """

    def __init__(self, spi_connection, **kwargs):
        super(OPCN3, self).__init__(spi_connection, model='N3', **kwargs)

       ## firmware_min = 0.   # Minimum firmware version supported
       ## firmware_max = 9999.   # Maximum firmware version supported

       ## if self.firmware['major'] < firmware_min or self.firmware['major'] > firmware_max:
       ##     logger.error("Firmware version is invalid for this device.")

       ## print self.firmware['major']

       ## raise FirmwareVersionError("Your firmware is not yet supported. Only versions 14-18 ...")

    def fan_on(self):
        
        # SEND COMMAND BYTE 0x03
        a = self.cnxn.xfer([0x03])[0]
        print "CMD: ", hex(a), a
        while a is not int('0x31',16): #[0xf3]: #49
            sleep(3)
            a = self.cnxn.xfer([0x03])[0]
            print "CMD: ", hex(a), a
        sleep(0.02) # >10ms <100ms
        a = self.cnxn.xfer([0x03])[0]
        sleep(0.02)
        
        # SEND 0x03 TO SET FAN ON
        a=int('0x03',16)
        print "sending ", "{0:08b}".format(a)
        a = self.cnxn.xfer([0x03])[0]
        print "FAN_ON: ", hex(a), a

    def fan_off(self):
        
        # SEND COMMAND BYTE 0x03
        a = self.cnxn.xfer([0x03])[0]
        print "CMD: ", hex(a), a
        while a is not int('0x31',16): #[0xf3]: #49
            sleep(3)
            a = self.cnxn.xfer([0x03])[0]
            print "CMD: ", hex(a), a
        sleep(0.02) # >10ms <100ms
        a = self.cnxn.xfer([0x03])[0]
        sleep(0.02)

        
        # SEND 0x02 TO SET FAN OFF
        a=int('0x02',16)
        print "sending ", "{0:08b}".format(a)
        a = self.cnxn.xfer([0x02])[0]
        print "FAN_OFF: ", hex(a), a

        
    def laser_on(self):
        
        # SEND COMMAND BYTE 0x03
        a = self.cnxn.xfer([0x03])[0]
        print "CMD: ", hex(a), a
        while a is not int('0x31',16): #[0xf3]: #49
            sleep(3)
            a = self.cnxn.xfer([0x03])[0]
            print "CMD: ", hex(a), a
        sleep(0.02) # >10ms <100ms
        a = self.cnxn.xfer([0x03])[0]
        sleep(0.02)        

        # SEND 0x07 TO SET FAN ON
        a=int('0x07',16)
        print "sending ", "{0:08b}".format(a)
        a = self.cnxn.xfer([0x07])[0]
        print "LASER_ON: ", hex(a), a

    def laser_off(self):
        
        # SEND COMMAND BYTE 0x03
        a = self.cnxn.xfer([0x03])[0]
        print "CMD: ", hex(a), a
        while a is not int('0x31',16): #[0xf3]: #49
            sleep(3)
            a = self.cnxn.xfer([0x03])[0]
            print "CMD: ", hex(a), a
        sleep(0.02) # >10ms <100ms
        a = self.cnxn.xfer([0x03])[0]
        sleep(0.02)        


        # SEND 0x06 TO SET LASER OFF
        a=int('0x06',16)
        print "sending ", "{0:08b}".format(a)
        a = self.cnxn.xfer([0x06])[0]        
        print "LASER_OFF: ", hex(a), a
        
    def on(self):
        """Turn ON the OPC (fan and laser)
        :rtype: boolean
        :Example:
        >>> alpha.on()
        True
        """

        # SEND COMMAND BYTE 0x03
        a = self.cnxn.xfer([0x03])[0]
        #print "CMD: ", hex(a), a
        while a is not int('0x31',16): #[0xf3]: #49
            sleep(3)
            a = self.cnxn.xfer([0x03])[0]
            #print "CMD: ", hex(a), a
        sleep(0.02) # >10ms <100ms
        a1 = self.cnxn.xfer([0x03])[0]
        sleep(0.02)
        
        # SEND 0x03 TO SET FAN ON
        a=int('0x03',16)
        #print "sending ", "{0:08b}".format(a)
        b1 = self.cnxn.xfer([0x03])[0]
        #print "FAN_ON: ", hex(b1), b1

        sleep(1)
        
        # SEND COMMAND BYTE 0x03
        a = self.cnxn.xfer([0x03])[0]
        #print "CMD: ", hex(a), a
        while a is not int('0x31',16): #[0xf3]: #49
            sleep(3)
            a = self.cnxn.xfer([0x03])[0]
            #print "CMD: ", hex(a), a
        sleep(0.02) # >10ms <100ms
        a2 = self.cnxn.xfer([0x03])[0]
        sleep(0.02)        

        # SEND 0x07 TO SET FAN ON
        a=int('0x07',16)
        #print "sending ", "{0:08b}".format(a)
        b2 = self.cnxn.xfer([0x07])[0]
        #print "LASER_ON: ", hex(b2), b2

        return True if a1 == 0xF3 and b1 == 0x03 and a2 == 0xF3 and b2 == 0x03 else False
 
    def off(self):
        """Turn OFF the OPC (fan and laser)
        :rtype: boolean
        :Example:
        >>> alpha.off()
        True
        """

        # SEND COMMAND BYTE 0x03
        a = self.cnxn.xfer([0x03])[0]
        #print "CMD: ", hex(a), a
        while a is not int('0x31',16): #[0xf3]: #49
            sleep(3)
            a = self.cnxn.xfer([0x03])[0]
            #print "CMD: ", hex(a), a
        sleep(0.02) # >10ms <100ms
        a1 = self.cnxn.xfer([0x03])[0]
        sleep(0.02)

        # SEND 0x06 TO SET LASER OFF
        a=int('0x06',16)
        #print "sending ", "{0:08b}".format(a)
        b1 = self.cnxn.xfer([0x06])[0]        
        #print "LASER_OFF: ", hex(b1), b1

        sleep(1)
        
        # SEND COMMAND BYTE 0x03
        a = self.cnxn.xfer([0x03])[0]
        #print "CMD: ", hex(a), a
        while a is not int('0x31',16): #[0xf3]: #49
            sleep(3)
            a = self.cnxn.xfer([0x03])[0]
            print "CMD: ", hex(a), a
        sleep(0.02) # >10ms <100ms
        a2 = self.cnxn.xfer([0x03])[0]
        sleep(0.02)
   
        # SEND 0x02 TO SET FAN OFF
        a=int('0x02',16)
        #print "sending ", "{0:08b}".format(a)
        b2 = self.cnxn.xfer([0x02])[0]
        #print "FAN_OFF: ", hex(b2), b2

        return True if a1 == 0xF3 and b1 == 0x03 and a2 == 0xF3 and b2 == 0x03 else False
        
    def read_pot_status(self):
        """Read the status of the digital pot. Firmware v18+ only.
        The return value is a dictionary containing the following as
        unsigned 8-bit integers: FanON, LaserON, FanDACVal, LaserDACVal.
        :rtype: dict
        :Example:
        >>> alpha.read_pot_status()
        {
            'LaserDACVal': 230,
            'FanDACVal': 255,
            'FanON': 0,
            'LaserON': 0
        }
        """
        # Send the command byte and wait 10 ms
        a = self.cnxn.xfer([0x13])[0]
        sleep(0.02)
        a = self.cnxn.xfer([0x13])[0]
        sleep(0.02)
        
        # Build an array of the results
        res = []
        for i in range(6):
            res.append(self.cnxn.xfer([0x00])[0])
        sleep(0.1)
        return {
            'FanON':        res[0],
            'LaserON':      res[1],
            'FanDACVal':    res[2],
            'LaserDACVal':  res[3],
            'LaserSwitch':  res[4],
            'GainToggle':  res[5]
            }


    def histogram(self, number_concentration=True):
        """Read and reset the histogram. As of v1.3.0, histogram
        values are reported in particle number concentration (#/cc) by default.
        :param number_concentration: If true, histogram bins are reported in number concentration vs. raw values.
        :type number_concentration: boolean
        :rtype: dictionary
        :Example:
        >>> alpha.histogram()
        {
            'Temperature': None, 'Pressure': None, 'Bin 0': 0, 'Bin 1': 0, 'Bin 2': 0, ... 'Bin 15': 0,
            'SFR': 3.700, 'Bin1MToF': 0, 'Bin3MToF': 0, 'Bin5MToF': 0, 'Bin7MToF': 0, 'PM1': 0.0, 'PM2.5': 0.0,
            'PM10': 0.0, 'Sampling Period': 2.345, 'Checksum': 0
        }
        """
        resp = []
        data = {}

        # Send the command byte
        a=0
        b=0
        arep=int('0x31',16)
        brep=int('0xf3',16)
        while a is not arep or b is not brep:
            a = self.cnxn.xfer([0x30])[0]
            #print hex(a), a
            sleep(0.02)
            b = self.cnxn.xfer([0x30])[0]
            #print hex(b), b

        # Wait 20 ms
        sleep(20e-3)
        #a = self.cnxn.xfer([0x30])[0]

        # read the histogram
        for i in range(86):
            #r = self.cnxn.xfer([0x00])[0]
            r = self.cnxn.xfer([0x30])[0]
            resp.append(r)
            sleep(10e-3)

        #print resp
            
        # convert to real things and store in dictionary!
        data['Bin 0']           = self._16bit_unsigned(resp[0], resp[1])
        data['Bin 1']           = self._16bit_unsigned(resp[2], resp[3])
        data['Bin 2']           = self._16bit_unsigned(resp[4], resp[5])
        data['Bin 3']           = self._16bit_unsigned(resp[6], resp[7])
        data['Bin 4']           = self._16bit_unsigned(resp[8], resp[9])
        data['Bin 5']           = self._16bit_unsigned(resp[10], resp[11])
        data['Bin 6']           = self._16bit_unsigned(resp[12], resp[13])
        data['Bin 7']           = self._16bit_unsigned(resp[14], resp[15])
        data['Bin 8']           = self._16bit_unsigned(resp[16], resp[17])
        data['Bin 9']           = self._16bit_unsigned(resp[18], resp[19])
        data['Bin 10']          = self._16bit_unsigned(resp[20], resp[21])
        data['Bin 11']          = self._16bit_unsigned(resp[22], resp[23])
        data['Bin 12']          = self._16bit_unsigned(resp[24], resp[25])
        data['Bin 13']          = self._16bit_unsigned(resp[26], resp[27])
        data['Bin 14']          = self._16bit_unsigned(resp[28], resp[29])
        data['Bin 15']          = self._16bit_unsigned(resp[30], resp[31])
        data['Bin 16']          = self._16bit_unsigned(resp[32], resp[33])
        data['Bin 17']          = self._16bit_unsigned(resp[34], resp[35])
        data['Bin 18']          = self._16bit_unsigned(resp[36], resp[37])
        data['Bin 19']          = self._16bit_unsigned(resp[38], resp[39])
        data['Bin 20']          = self._16bit_unsigned(resp[40], resp[41])
        data['Bin 21']          = self._16bit_unsigned(resp[42], resp[43])
        data['Bin 22']          = self._16bit_unsigned(resp[44], resp[45])
        data['Bin 23']          = self._16bit_unsigned(resp[46], resp[47])

        data['Bin1 MToF']       = self._calculate_mtof(resp[48])
        data['Bin3 MToF']       = self._calculate_mtof(resp[49])
        data['Bin5 MToF']       = self._calculate_mtof(resp[50])
        data['Bin7 MToF']       = self._calculate_mtof(resp[51])

        data['Sampling Period'] = self._calculate_period_uint(resp[52],resp[53])
        data['Sample Flow Rate'] = self._calculate_flowrate(resp[54],resp[55])
        data['Temperature']     = self._calculate_temp_uint(resp[56],resp[57])
        data['Relative humidity']     = self._calculate_hum_uint(resp[58],resp[59])

        data['PM_A']     = self._calculate_float(resp[60:64])
        data['PM_B']     = self._calculate_float(resp[64:68])
        data['PM_C']     = self._calculate_float(resp[68:72])

        data['Reject count Glitch'] = self._16bit_unsigned(resp[72], resp[73])
        data['Reject count LongTOF'] = self._16bit_unsigned(resp[74], resp[75])
        data['Reject count Ratio'] = self._16bit_unsigned(resp[76], resp[77])
        data['Reject Count OutOfRange'] = self._16bit_unsigned(resp[78], resp[79])
        data['Fan rev count'] = self._16bit_unsigned(resp[80], resp[81])
        data['Laser status'] = self._16bit_unsigned(resp[82], resp[83])
        data['Checksum'] = self._16bit_unsigned(resp[84], resp[85])


        # Calculate the sum of the histogram bins
        histogram_sum = data['Bin 0'] + data['Bin 1'] + data['Bin 2']   + \
                data['Bin 3'] + data['Bin 4'] + data['Bin 5'] + data['Bin 6']   + \
                data['Bin 7'] + data['Bin 8'] + data['Bin 9'] + data['Bin 10']  + \
                data['Bin 11'] + data['Bin 12'] + data['Bin 13'] + data['Bin 14'] + \
                data['Bin 15'] + data['Bin 16'] + data['Bin 17'] + data['Bin 18'] + \
                data['Bin 19'] + data['Bin 20'] + data['Bin 21'] + data['Bin 22'] + \
                data['Bin 23']

        # Check that checksum and the least significant bits of the sum of histogram bins
        # are equivilant
        #if (histogram_sum & 0x0000FFFF) != data['Checksum']:
        if (self._MODBUS_CalcCRC(resp,84) != data['Checksum']):
            print "CHECKSUM: ", histogram_sum, data['Checksum']
            logger.warning("Data transfer was incomplete")
            ##return None

        # If histogram is true, convert histogram values to number concentration
        if number_concentration is True:
            _conv_ = data['Sample Flow Rate'] * data['Sampling Period'] # Divider in units of ml (cc)

            data['Bin 0']   = data['Bin 0'] / _conv_
            data['Bin 1']   = data['Bin 1'] / _conv_
            data['Bin 2']   = data['Bin 2'] / _conv_
            data['Bin 3']   = data['Bin 3'] / _conv_
            data['Bin 4']   = data['Bin 4'] / _conv_
            data['Bin 5']   = data['Bin 5'] / _conv_
            data['Bin 6']   = data['Bin 6'] / _conv_
            data['Bin 7']   = data['Bin 7'] / _conv_
            data['Bin 8']   = data['Bin 8'] / _conv_
            data['Bin 9']   = data['Bin 9'] / _conv_
            data['Bin 10']  = data['Bin 10'] / _conv_
            data['Bin 11']  = data['Bin 11'] / _conv_
            data['Bin 12']  = data['Bin 12'] / _conv_
            data['Bin 13']  = data['Bin 13'] / _conv_
            data['Bin 14']  = data['Bin 14'] / _conv_
            data['Bin 15']  = data['Bin 15'] / _conv_
            data['Bin 16']  = data['Bin 16'] / _conv_
            data['Bin 17']  = data['Bin 17'] / _conv_
            data['Bin 18']  = data['Bin 18'] / _conv_
            data['Bin 19']  = data['Bin 19'] / _conv_
            data['Bin 20']  = data['Bin 20'] / _conv_
            data['Bin 21']  = data['Bin 21'] / _conv_
            data['Bin 22']  = data['Bin 22'] / _conv_
            data['Bin 23']  = data['Bin 23'] / _conv_


        #print "Bin  0:", data['Bin 0']
        #print "Bin  1:", data['Bin 1']
        #print "Bin  2:", data['Bin 2']
        #print "Bin  3:", data['Bin 3']
        #print "Bin  4:", data['Bin 4']
        #print "Bin  5:", data['Bin 5']
        #print "Bin  6:", data['Bin 6']
        #print "Bin  7:", data['Bin 7']
        #print "Bin  8:", data['Bin 8']
        #print "Bin  9:", data['Bin 9']
        #print "Bin 10:", data['Bin 10']
        #print "Bin 11:", data['Bin 11']
        #print "Bin 12:", data['Bin 12']
        #print "Bin 13:", data['Bin 13']
        #print "Bin 14:", data['Bin 14']
        #print "Bin 15:", data['Bin 15']
        #print "Bin 16:", data['Bin 16']
        #print "Bin 17:", data['Bin 17']
        #print "Bin 18:", data['Bin 18']
        #print "Bin 19:", data['Bin 19']
        #print "Bin 20:", data['Bin 20']
        #print "Bin 21:", data['Bin 21']
        #print "Bin 22:", data['Bin 22']
        #print "Bin 23:", data['Bin 23']
            
        #print "PM  1.0:", data['PM_A']
        #print "PM  2.5:", data['PM_B']
        #print "PM 10.0:", data['PM_C']        

        sleep(0.1)

	#data = numpy.sort(data.dtype.names)
        #data = sorted(data.iterkeys()

        return data

    def sn(self):
        """Read the Serial Number string. This method is only available on OPC-N2
        firmware versions 18+.
        :rtype: string
        :Example:
        >>> alpha.sn()
        'OPC-N2 123456789'
        """
        string = []

        # Send the command byte and sleep for 9 ms
        a = self.cnxn.xfer([0x10])
        sleep(0.02)
        b = self.cnxn.xfer([0x10])
        sleep(0.02)

        # Read the info string by sending 60 empty bytes
        for i in range(60):
            resp = self.cnxn.xfer([0x10])[0]
            string.append(chr(resp))

        sleep(0.1)

        return ''.join(string)

    def config(self):
        """Read the configuration variables and returns them as a dictionary

        :rtype: dictionary

        :Example:

        >>> alpha.config()
        {
            'BPD 13': 1.6499,
            'BPD 12': 1.6499,
            'BPD 11': 1.6499,
            'BPD 10': 1.6499,
            'BPD 15': 1.6499,
            'BPD 14': 1.6499,
            'BSVW 15': 1.0,
            ...
        }
        """
        config  = []
        data    = {}

        # Send the command byte and sleep for 10 ms
        self.cnxn.xfer([0x3C])
        sleep(10e-3)

        # Read the config variables by sending 167 empty bytes
        for i in range(168):
           # resp = self.cnxn.xfer([0x00])[0]
            resp = self.cnxn.xfer([0x3C])[0]
            config.append(resp)
            sleep(10e-3)

        # Add the bin boundaries ADC (BB) to the dictionary of data [bytes 0-48]
        for i in range(0, 25):
            data["BB {0}".format(i)] = self._16bit_unsigned(config[2*i+1], config[2*i + 2])

        # Add the Bin Boundaries diameter (um) (BBD) [bytes 49-99]
        for i in range(0, 25):
            data["BBD {0}".format(i)] = self._16bit_unsigned(config[2*i+51], config[2*i + 52])

        # Add the Bin Weightings (BW) [bytes 100-146]
        for i in range(0, 24):
            data["BW {0}".format(i)] = self._16bit_unsigned(config[2*i+102], config[2*i + 103])

        # Add the PM diameter A,B,C (M_A) [bytes 147-152]
        data["M_A"] = self._16bit_unsigned(config[149], config[150])/100.
        data["M_B"] = self._16bit_unsigned(config[151], config[152])/100.
        data["M_C"] = self._16bit_unsigned(config[153], config[154])/100.

        # Add the Maximum Time Of Flight (MaxTOF) ...  [bytes 153-166]
        data["MaxTOF"] = self._16bit_unsigned(config[155], config[156])
        data["AMSamplingIntervalCount"] = self._16bit_unsigned(config[157], config[158])
        data["AMIdleIntervalCount"] = self._16bit_unsigned(config[159], config[160])
        data["AMMaxDataArraysInFile"] = self._16bit_unsigned(config[161], config[162])
        data["AMOnlySavePMData"] = config[163]
        data["AMFanOnInIdle"] = config[164]
        data["AMLaserOnInIdle"] = config[165]
        data["TOFtoSFR"] = config[166] #Time of Flight to Sample Flow Rate conversion factor
        data["PVP"] = config[167] #Particle Validation Period
        data["BWI"] = config[166] #Bin Weighting Index (0-9), represents the index of the preset bin weighting to use

        sleep(0.1)

        return data

###################################################################################
class OPCN2(_OPC):
    """Create an instance of the Alphasene OPC-N2. Currently supported by firmware
    versions 14-18. opc.OPCN2 inherits from the opc.OPC parent class.

    :param spi_connection: The spidev instance for the SPI connection.

    :type spi_connection: spidev.SpiDev

    :rtype: opc.OPCN2

    :raises: opc.exceptions.FirmwareVersionError

    :Example:

    >>> alpha = opc.OPCN2(spi)
    >>> alpha
    Alphasense OPC-N2v18.2
    """
    def __init__(self, spi_connection, **kwargs):
        super(OPCN2, self).__init__(spi_connection, model='N2', **kwargs)

        firmware_min = 14.   # Minimum firmware version supported
        firmware_max = 18.   # Maximum firmware version supported

        if self.firmware['major'] < firmware_min or self.firmware['major'] > firmware_max:
            logger.error("Firmware version is invalid for this device.")

            raise FirmwareVersionError("Your firmware is not yet supported. Only versions 14-18 are currently supported.")

    def on(self):
        """Turn ON the OPC (fan and laser)

        :rtype: boolean

        :Example:

        >>> alpha.on()
        True
        """
        b1 = self.cnxn.xfer([0x03])[0]          # send the command byte
        sleep(9e-3)                             # sleep for 9 ms
        b2, b3 = self.cnxn.xfer([0x00, 0x01])   # send the following byte
        sleep(0.1)

        return True if b1 == 0xF3 and b2 == 0x03 else False

    def off(self):
        """Turn OFF the OPC (fan and laser)

        :rtype: boolean

        :Example:

        >>> alpha.off()
        True
        """
        b1 = self.cnxn.xfer([0x03])[0]          # send the command byte
        sleep(9e-3)                             # sleep for 9 ms
        b2 = self.cnxn.xfer([0x01])[0]          # send the following two bytes
        sleep(0.1)

        return True if b1 == 0xF3 and b2 == 0x03 else False

    def config(self):
        """Read the configuration variables and returns them as a dictionary

        :rtype: dictionary

        :Example:

        >>> alpha.config()
        {
            'BPD 13': 1.6499,
            'BPD 12': 1.6499,
            'BPD 11': 1.6499,
            'BPD 10': 1.6499,
            'BPD 15': 1.6499,
            'BPD 14': 1.6499,
            'BSVW 15': 1.0,
            ...
        }
        """
        config  = []
        data    = {}

        # Send the command byte and sleep for 10 ms
        self.cnxn.xfer([0x3C])
        sleep(10e-3)

        # Read the config variables by sending 256 empty bytes
        for i in range(256):
            resp = self.cnxn.xfer([0x00])[0]
            config.append(resp)

        # Add the bin bounds to the dictionary of data [bytes 0-29]
        for i in range(0, 15):
            data["Bin Boundary {0}".format(i)] = self._16bit_unsigned(config[2*i], config[2*i + 1])

        # Add the Bin Particle Volumes (BPV) [bytes 32-95]
        for i in range(0, 16):
            data["BPV {0}".format(i)] = self._calculate_float(config[4*i + 32:4*i + 36])

        # Add the Bin Particle Densities (BPD) [bytes 96-159]
        for i in range(0, 16):
            data["BPD {0}".format(i)] = self._calculate_float(config[4*i + 96:4*i + 100])

        # Add the Bin Sample Volume Weight (BSVW) [bytes 160-223]
        for i in range(0, 16):
            data["BSVW {0}".format(i)] = self._calculate_float(config[4*i + 160: 4*i + 164])

        # Add the Gain Scaling Coefficient (GSC) and sample flow rate (SFR)
        data["GSC"] = self._calculate_float(config[224:228])
        data["SFR"] = self._calculate_float(config[228:232])

        # Add laser dac (LDAC) and Fan dac (FanDAC)
        data["LaserDAC"]    = config[232]
        data["FanDAC"]      = config[233]

        # If past firmware 15, add other things
        if self.firmware['major'] > 15.:
            data['TOF_SFR'] = config[234]

        sleep(0.1)

        return data

    @requires_firmware(18.)
    def config2(self):
        """Read the second set of configuration variables and return as a dictionary.

        **NOTE: This method is supported by firmware v18+.**

        :rtype: dictionary

        :Example:

        >>> a.config2()
        {
            'AMFanOnIdle': 0,
            'AMIdleIntervalCount': 0,
            'AMMaxDataArraysInFile': 61798,
            'AMSamplingInterval': 1,
            'AMOnlySavePMData': 0,
            'AMLaserOnIdle': 0
        }
        """
        config  = []
        data    = {}

        # Send the command byte and sleep for 10 ms
        self.cnxn.xfer([0x3D])
        sleep(10e-3)

        # Read the config variables by sending 256 empty bytes
        for i in range(9):
            resp = self.cnxn.xfer([0x00])[0]
            config.append(resp)

        data["AMSamplingInterval"]      = self._16bit_unsigned(config[0], config[1])
        data["AMIdleIntervalCount"]     = self._16bit_unsigned(config[2], config[3])
        data['AMFanOnIdle']             = config[4]
        data['AMLaserOnIdle']           = config[5]
        data['AMMaxDataArraysInFile']   = self._16bit_unsigned(config[6], config[7])
        data['AMOnlySavePMData']        = config[8]

        sleep(0.1)

        return data

    def write_config_variables(self, config_vars):
        """ Write configuration variables to non-volatile memory.

        **NOTE: This method is currently a placeholder and is not implemented.**

        :param config_vars: dictionary containing the configuration variables

        :type config_vars: dictionary
        """
        logger.warning("This method has not yet been implemented yet.")

        return

    @requires_firmware(18.)
    def write_config_variables2(self, config_vars):
        """ Write configuration variables 2 to non-volatile memory.

        **NOTE: This method is currently a placeholder and is not implemented.**
        **NOTE: This method is supported by firmware v18+.**

        :param config_vars: dictionary containing the configuration variables

        :type config_vars: dictionary
        """

        logger.warning("This method has not yet been implemented.")

        return

    def histogram(self, number_concentration=True):
        """Read and reset the histogram. As of v1.3.0, histogram
        values are reported in particle number concentration (#/cc) by default.

        :param number_concentration: If true, histogram bins are reported in number concentration vs. raw values.

        :type number_concentration: boolean

        :rtype: dictionary

        :Example:

        >>> alpha.histogram()
        {
            'Temperature': None,
            'Pressure': None,
            'Bin 0': 0,
            'Bin 1': 0,
            'Bin 2': 0,
            ...
            'Bin 15': 0,
            'SFR': 3.700,
            'Bin1MToF': 0,
            'Bin3MToF': 0,
            'Bin5MToF': 0,
            'Bin7MToF': 0,
            'PM1': 0.0,
            'PM2.5': 0.0,
            'PM10': 0.0,
            'Sampling Period': 2.345,
            'Checksum': 0
        }
        """
        resp = []
        data = {}

        # Send the command byte
        self.cnxn.xfer([0x30])

        # Wait 10 ms
        sleep(10e-3)

        # read the histogram
        for i in range(62):
            r = self.cnxn.xfer([0x00])[0]
            resp.append(r)

        # convert to real things and store in dictionary!
        data['Bin 0']           = self._16bit_unsigned(resp[0], resp[1])
        data['Bin 1']           = self._16bit_unsigned(resp[2], resp[3])
        data['Bin 2']           = self._16bit_unsigned(resp[4], resp[5])
        data['Bin 3']           = self._16bit_unsigned(resp[6], resp[7])
        data['Bin 4']           = self._16bit_unsigned(resp[8], resp[9])
        data['Bin 5']           = self._16bit_unsigned(resp[10], resp[11])
        data['Bin 6']           = self._16bit_unsigned(resp[12], resp[13])
        data['Bin 7']           = self._16bit_unsigned(resp[14], resp[15])
        data['Bin 8']           = self._16bit_unsigned(resp[16], resp[17])
        data['Bin 9']           = self._16bit_unsigned(resp[18], resp[19])
        data['Bin 10']          = self._16bit_unsigned(resp[20], resp[21])
        data['Bin 11']          = self._16bit_unsigned(resp[22], resp[23])
        data['Bin 12']          = self._16bit_unsigned(resp[24], resp[25])
        data['Bin 13']          = self._16bit_unsigned(resp[26], resp[27])
        data['Bin 14']          = self._16bit_unsigned(resp[28], resp[29])
        data['Bin 15']          = self._16bit_unsigned(resp[30], resp[31])
        data['Bin1 MToF']       = self._calculate_mtof(resp[32])
        data['Bin3 MToF']       = self._calculate_mtof(resp[33])
        data['Bin5 MToF']       = self._calculate_mtof(resp[34])
        data['Bin7 MToF']       = self._calculate_mtof(resp[35])

        # Bins associated with firmware versions 14 and 15(?)
        if self.firmware['version'] < 16.:
            data['Temperature']     = self._calculate_temp(resp[36:40])
            data['Pressure']        = self._calculate_pressure(resp[40:44])
            data['Sampling Period'] = self._calculate_period(resp[44:48])
            data['Checksum']        = self._16bit_unsigned(resp[48], resp[49])
            data['PM1']             = self._calculate_float(resp[50:54])
            data['PM2.5']           = self._calculate_float(resp[54:58])
            data['PM10']            = self._calculate_float(resp[58:])

        else:
            data['SFR']             = self._calculate_float(resp[36:40])

            # Alright, we don't know whether it is temp or pressure since it switches..
            tmp = self._calculate_pressure(resp[40:44])
            if tmp > 98000:
                data['Temperature'] = None
                data['Pressure']    = tmp
            else:
                tmp = self._calculate_temp(resp[40:44])
                if tmp < 500:
                    data['Temperature'] = tmp
                    data['Pressure']    = None
                else:
                    data['Temperature'] = None
                    data['Pressure']    = None

            data['Sampling Period'] = self._calculate_float(resp[44:48])
            data['Checksum']        = self._16bit_unsigned(resp[48], resp[49])
            data['PM1']             = self._calculate_float(resp[50:54])
            data['PM2.5']           = self._calculate_float(resp[54:58])
            data['PM10']            = self._calculate_float(resp[58:])

        # Calculate the sum of the histogram bins
        histogram_sum = data['Bin 0'] + data['Bin 1'] + data['Bin 2']   + \
                data['Bin 3'] + data['Bin 4'] + data['Bin 5'] + data['Bin 6']   + \
                data['Bin 7'] + data['Bin 8'] + data['Bin 9'] + data['Bin 10']  + \
                data['Bin 11'] + data['Bin 12'] + data['Bin 13'] + data['Bin 14'] + \
                data['Bin 15']

        # Check that checksum and the least significant bits of the sum of histogram bins
        # are equivilant
        if (histogram_sum & 0x0000FFFF) != data['Checksum']:
            logger.warning("Data transfer was incomplete")
            return None

        # If histogram is true, convert histogram values to number concentration
        if number_concentration is True:
            _conv_ = data['SFR'] * data['Sampling Period'] # Divider in units of ml (cc)

            data['Bin 0']   = data['Bin 0'] / _conv_
            data['Bin 1']   = data['Bin 1'] / _conv_
            data['Bin 2']   = data['Bin 2'] / _conv_
            data['Bin 3']   = data['Bin 3'] / _conv_
            data['Bin 4']   = data['Bin 4'] / _conv_
            data['Bin 5']   = data['Bin 5'] / _conv_
            data['Bin 6']   = data['Bin 6'] / _conv_
            data['Bin 7']   = data['Bin 7'] / _conv_
            data['Bin 8']   = data['Bin 8'] / _conv_
            data['Bin 9']   = data['Bin 9'] / _conv_
            data['Bin 10']  = data['Bin 10'] / _conv_
            data['Bin 11']  = data['Bin 11'] / _conv_
            data['Bin 12']  = data['Bin 12'] / _conv_
            data['Bin 13']  = data['Bin 13'] / _conv_
            data['Bin 14']  = data['Bin 14'] / _conv_
            data['Bin 15']  = data['Bin 15'] / _conv_

        sleep(0.1)

        return data

    def save_config_variables(self):
        """Save the configuration variables in non-volatile memory. This method
        should be used in conjuction with *write_config_variables*.

        :rtype: boolean

        :Example:

        >>> alpha.save_config_variables()
        True
        """
        command = 0x43
        byte_list = [0x3F, 0x3C, 0x3F, 0x3C, 0x43]
        success = [0xF3, 0x43, 0x3F, 0x3C, 0x3F, 0x3C]
        resp = []

        # Send the command byte and then wait for 10 ms
        r = self.cnxn.xfer([command])[0]
        sleep(10e-3)

        # append the response of the command byte to the List
        resp.append(r)

        # Send the rest of the config bytes
        for each in byte_list:
            r = self.cnxn.xfer([each])[0]
            resp.append(r)

        sleep(0.1)

        return True if resp == success else False

    def _enter_bootloader_mode(self):
        """Enter bootloader mode. Must be issued prior to writing
        configuration variables to non-volatile memory.

        :rtype: boolean

        :Example:

        >>> alpha._enter_bootloader_mode()
        True
        """

        return True if self.cnxn.xfer(0x41)[0] == 0xF3 else False

    def set_fan_power(self, power):
        """Set only the Fan power.

        :param power: Fan power value as an integer between 0-255.

        :type power: int

        :rtype: boolean

        :Example:

        >>> alpha.set_fan_power(255)
        True
        """
        # Check to make sure the value is a single byte
        if power > 255:
            raise ValueError("The fan power should be a single byte (0-255).")

        # Send the command byte and wait 10 ms
        a = self.cnxn.xfer([0x42])[0]
        sleep(10e-3)

        # Send the next two bytes
        b = self.cnxn.xfer([0x00])[0]
        c = self.cnxn.xfer([power])[0]

        sleep(0.1)

        return True if a == 0xF3 and b == 0x42 and c == 0x00 else False

    def set_laser_power(self, power):
        """Set the laser power only.

        :param power: Laser power as a value between 0-255.

        :type power: int

        :rtype: boolean

        :Example:

        >>> alpha.set_laser_power(230)
        True
        """

        # Check to make sure the value is a single byte
        if power > 255:
            raise ValueError("Laser Power should be a single byte (0-255).")

        # Send the command byte and wait 10 ms
        a = self.cnxn.xfer([0x42])[0]
        sleep(10e-3)

        # Send the next two bytes
        b = self.cnxn.xfer([0x01])[0]
        c = self.cnxn.xfer([power])[0]

        sleep(0.1)

        return True if a == 0xF3 and b == 0x42 and c == 0x01 else False

    def toggle_laser(self, state):
        """Toggle the power state of the laser.

        :param state: Boolean state of the laser

        :type state: boolean

        :rtype: boolean

        :Example:

        >>> alpha.toggle_laser(True)
        True
        """

        # Send the command byte and wait 10 ms
        a = self.cnxn.xfer([0x03])[0]

        sleep(10e-3)

        # If state is true, turn the laser ON, else OFF
        if state:
            b = self.cnxn.xfer([0x02])[0]
        else:
            b = self.cnxn.xfer([0x03])[0]

        sleep(0.1)

        return True if a == 0xF3 and b == 0x03 else False

    def toggle_fan(self, state):
        """Toggle the power state of the fan.

        :param state: Boolean state of the fan

        :type state: boolean

        :rtype: boolean

        :Example:

        >>> alpha.toggle_fan(False)
        True
        """

        # Send the command byte and wait 10 ms
        a = self.cnxn.xfer([0x03])[0]

        sleep(10e-3)

        # If state is true, turn the fan ON, else OFF
        if state:
            b = self.cnxn.xfer([0x04])[0]
        else:
            b = self.cnxn.xfer([0x05])[0]

        sleep(0.1)

        return True if a == 0xF3 and b == 0x03 else False

    @requires_firmware(18.)
    def read_pot_status(self):
        """Read the status of the digital pot. Firmware v18+ only.
        The return value is a dictionary containing the following as
        unsigned 8-bit integers: FanON, LaserON, FanDACVal, LaserDACVal.

        :rtype: dict

        :Example:

        >>> alpha.read_pot_status()
        {
            'LaserDACVal': 230,
            'FanDACVal': 255,
            'FanON': 0,
            'LaserON': 0
        }
        """
        # Send the command byte and wait 10 ms
        a = self.cnxn.xfer([0x13])[0]

        sleep(10e-3)

        # Build an array of the results
        res = []
        for i in range(4):
            res.append(self.cnxn.xfer([0x00])[0])

        sleep(0.1)

        return {
            'FanON':        res[0],
            'LaserON':      res[1],
            'FanDACVal':    res[2],
            'LaserDACVal':  res[3]
            }

    @requires_firmware(18.)
    def sn(self):
        """Read the Serial Number string. This method is only available on OPC-N2
        firmware versions 18+.

        :rtype: string

        :Example:

        >>> alpha.sn()
        'OPC-N2 123456789'
        """
        string = []

        # Send the command byte and sleep for 9 ms
        self.cnxn.xfer([0x10])
        sleep(9e-3)

        # Read the info string by sending 60 empty bytes
        for i in range(60):
            resp = self.cnxn.xfer([0x00])[0]
            string.append(chr(resp))

        sleep(0.1)

        return ''.join(string)

    @requires_firmware(18.)
    def write_sn(self):
        """Write the Serial Number string. This method is available for Firmware versions 18+.

        **NOTE: This method is currently a placeholder and is not implemented.**

        :param sn: string containing the serial number to write

        :type sn: string
        """

        return

    @requires_firmware(18.)
    def read_firmware(self):
        """Read the firmware version of the OPC-N2. Firmware v18+ only.

        :rtype: dict

        :Example:

        >>> alpha.read_firmware()
        {
            'major': 18,
            'minor': 2,
            'version': 18.2
        }
        """
        # Send the command byte and sleep for 9 ms
        self.cnxn.xfer([0x12])
        sleep(10e-3)

        self.firmware['major'] = self.cnxn.xfer([0x31])[0]
        self.firmware['minor'] = self.cnxn.xfer([0x31])[0]

        # Build the firmware version
        self.firmware['version'] = float('{}.{}'.format(self.firmware['major'], self.firmware['minor']))

        sleep(0.1)

        return self.firmware

    @requires_firmware(18.)
    def pm(self):
        """Read the PM data and reset the histogram

        **NOTE: This method is supported by firmware v18+.**

        :rtype: dictionary

        :Example:

        >>> alpha.pm()
        {
            'PM1': 0.12,
            'PM2.5': 0.24,
            'PM10': 1.42
        }
        """

        resp = []
        data = {}

        # Send the command byte
        self.cnxn.xfer([0x32])

        # Wait 10 ms
        sleep(10e-3)

        # read the histogram
        for i in range(12):
            r = self.cnxn.xfer([0x00])[0]
            resp.append(r)

        # convert to real things and store in dictionary!
        data['PM1']     = self._calculate_float(resp[0:4])
        data['PM2.5']   = self._calculate_float(resp[4:8])
        data['PM10']    = self._calculate_float(resp[8:])

        sleep(0.1)

        return data

class OPCN1(_OPC):
    """Create an instance of the Alphasene OPC-N1. opc.OPCN1 inherits from
    the opc.OPC parent class.

    :param spi_connection: The spidev instance for the SPI connection.

    :type spi_connection: spidev.SpiDev

    :rtype: opc.OPCN1

    :raises: FirmwareVersionError
    """
    def __init(self, spi_connection, **kwargs):
        super(OPCN1, self).__init__(spi_connection, model='N1', **kwargs)

    def on(self):
        """Turn ON the OPC (fan and laser)

        :returns: boolean success state
        """
        b1 = self.cnxn.xfer([0x0C])[0]          # send the command byte
        sleep(9e-3)                             # sleep for 9 ms

        return True if b1 == 0xF3 else False

    def off(self):
        """Turn OFF the OPC (fan and laser)

        :returns: boolean success state
        """
        b1 = self.cnxn.xfer([0x03])[0]          # send the command byte
        sleep(9e-3)                             # sleep for 9 ms

        return True if b1 == 0xF3 else False

    def read_gsc_sfr(self):
        """Read the gain-scaling-coefficient and sample flow rate.

        :returns: dictionary containing GSC and SFR
        """
        config  = []
        data    = {}

        # Send the command byte and sleep for 10 ms
        self.cnxn.xfer([0x33])
        sleep(10e-3)

        # Read the config variables by sending 256 empty bytes
        for i in range(8):
            resp = self.cnxn.xfer([0x00])[0]
            config.append(resp)

        data["GSC"] = self._calculate_float(config[0:4])
        data["SFR"] = self._calculate_float(config[4:])

        return data

    def read_bin_boundaries(self):
        """Return the bin boundaries.

        :returns: dictionary with 17 bin boundaries.
        """
        config  = []
        data    = {}

        # Send the command byte and sleep for 10 ms
        self.cnxn.xfer([0x33])
        sleep(10e-3)

        # Read the config variables by sending 256 empty bytes
        for i in range(30):
            resp = self.cnxn.xfer([0x00])[0]
            config.append(resp)

        # Add the bin bounds to the dictionary of data [bytes 0-29]
        for i in range(0, 14):
            data["Bin Boundary {0}".format(i)] = self._16bit_unsigned(config[2*i], config[2*i + 1])

        return data

    def write_gsc_sfr(self):
        """Write the gsc and sfr values

        **NOTE**: This method is currently a placeholder.
        """
        return

    def read_bin_particle_density(self):
        """Read the bin particle density

        :returns: float
        """
        config = []

        # Send the command byte and sleep for 10 ms
        self.cnxn.xfer([0x33])
        sleep(10e-3)

        # Read the config variables by sending 256 empty bytes
        for i in range(4):
            resp = self.cnxn.xfer([0x00])[0]
            config.append(resp)

        bpd = self._calculate_float(config)

        return bpd

    def write_bin_particle_density(self):
        """Write the bin particle density values to memory. This method is currently a
        placeholder.

        :returns: None
        """
        return

    def read_histogram(self):
        """Read and reset the histogram. The expected return is a dictionary
        containing the counts per bin, MToF for bins 1, 3, 5, and 7, temperature,
        pressure, the sampling period, the checksum, PM1, PM2.5, and PM10.

        **NOTE:** The sampling period for the OPCN1 seems to be incorrect.

        :returns: dictionary
        """
        resp = []
        data = {}

        # command byte
        command = 0x30

        # Send the command byte
        self.cnxn.xfer([command])

        # Wait 10 ms
        sleep(10e-3)

        # read the histogram
        for i in range(62):
            r = self.cnxn.xfer([0x00])[0]
            resp.append(r)

        # convert to real things and store in dictionary!
        data['Bin 0']           = self._16bit_unsigned(resp[0], resp[1])
        data['Bin 1']           = self._16bit_unsigned(resp[2], resp[3])
        data['Bin 2']           = self._16bit_unsigned(resp[4], resp[5])
        data['Bin 3']           = self._16bit_unsigned(resp[6], resp[7])
        data['Bin 4']           = self._16bit_unsigned(resp[8], resp[9])
        data['Bin 5']           = self._16bit_unsigned(resp[10], resp[11])
        data['Bin 6']           = self._16bit_unsigned(resp[12], resp[13])
        data['Bin 7']           = self._16bit_unsigned(resp[14], resp[15])
        data['Bin 8']           = self._16bit_unsigned(resp[16], resp[17])
        data['Bin 9']           = self._16bit_unsigned(resp[18], resp[19])
        data['Bin 10']          = self._16bit_unsigned(resp[20], resp[21])
        data['Bin 11']          = self._16bit_unsigned(resp[22], resp[23])
        data['Bin 12']          = self._16bit_unsigned(resp[24], resp[25])
        data['Bin 13']          = self._16bit_unsigned(resp[26], resp[27])
        data['Bin 14']          = self._16bit_unsigned(resp[28], resp[29])
        data['Bin 15']          = self._16bit_unsigned(resp[30], resp[31])
        data['Bin1 MToF']       = self._calculate_mtof(resp[32])
        data['Bin3 MToF']       = self._calculate_mtof(resp[33])
        data['Bin5 MToF']       = self._calculate_mtof(resp[34])
        data['Bin7 MToF']       = self._calculate_mtof(resp[35])
        data['Temperature']     = self._calculate_temp(resp[36:40])
        data['Pressure']        = self._calculate_pressure(resp[40:44])
        data['Sampling Period'] = self._calculate_period(resp[44:48])
        data['Checksum']        = self._16bit_unsigned(resp[48], resp[49])
        data['PM1']             = self._calculate_float(resp[50:54])
        data['PM2.5']           = self._calculate_float(resp[54:58])
        data['PM10']            = self._calculate_float(resp[58:])

        # Calculate the sum of the histogram bins
        histogram_sum = data['Bin 0'] + data['Bin 1'] + data['Bin 2']   + \
                data['Bin 3'] + data['Bin 4'] + data['Bin 5'] + data['Bin 6']   + \
                data['Bin 7'] + data['Bin 8'] + data['Bin 9'] + data['Bin 10']  + \
                data['Bin 11'] + data['Bin 12'] + data['Bin 13'] + data['Bin 14'] + \
                data['Bin 15']

        return data
