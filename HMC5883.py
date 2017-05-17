
from OmegaExpansion import onionI2C
import time
import math

class HMC5883:
    _scales = {
        0.88: [0, 0.73],
        1.30: [1, 0.92],
        1.90: [2, 1.22],
        2.50: [3, 1.52],
        4.00: [4, 2.27],
        4.70: [5, 2.56],
        5.60: [6, 3.03],
        8.10: [7, 4.35],
    }

    def __init__(self, gauss = 5.6, declination = (10,59)):
                
        # Get I2C bus
        self.i2c = onionI2C.OnionI2C()

        (degrees, minutes) = declination
        self._declDegrees = degrees
        self._declMinutes = minutes
        self._declination = (degrees + minutes/ 60) * math.pi /180

        (reg, self._scale) = self._scales[gauss]
        self.i2c.writeByte(0x1E, 0x00, 0x70) # 8 Average, 15 Hz, Normal
        self.i2c.writeByte(0x1E, 0x01, reg << 5) #Scale
        self.i2c.writeByte(0x1E, 0x02, 0x00) # COntinuous Measurement Mode
        
    def declination(self):
        return (self._declDegrees, self._declMinutes)

    def twos_complement(self, val, len):
        # Convert twos compliment to integer
        if (val & (1 << len - 1)):
            val = val - (1<<len)
        return val

    def _convert(self, data, offset):
        val = self.twos_complement(data[offset] << 8 | data[offset+1], 16)
        if val == -4096: return None
        return round(val * self._scale, 4)

    def axes(self):
        data = self.i2c.readBytes(self.address, 0x00, 10)
        #print map(hex, data)
        x = self._convert(data, 3)
        y = self._convert(data, 7)
        z = self._convert(data, 5)
        return (x,y,z)

    def heading(self):
        (x, y, z) = self.axes()
        headingRad = math.atan2(y, x)
        headingRad += self._declination

        # Correct for reversed heading
        if headingRad < 0:
            headingRad += 2 * math.pi

        # Check for wrap and compensate
        elif headingRad > 2 * math.pi:
            headingRad -= 2 * math.pi

        # Convert to degrees from radians
        headingDeg = headingRad * 180 / math.pi
        return headingDeg

    def degrees(self, headingDeg):
        degrees = math.floor(headingDeg)
        minutes = round((headingDeg - degrees) * 60)
        return (degrees, minutes)

    def __str__(self):
        (x, y, z) = self.axes()
        return "Axis X: " + str(x) + "\n" \
               "Axis Y: " + str(y) + "\n" \
               "Axis Z: " + str(z) + "\n" \
               "Declination: " + self.degrees(self.declination()) + "\n" \
               "Heading: " + self.degrees(self.heading()) + "\n"

if __name__ == "__main__":
    # http://magnetic-declination.com/
    compass = hmc5883l(gauss = 5.6, declination = (10,59))
    while True:
        #sys.stdout.write("\rHeading: " + str(compass.degrees(compass.heading())) + "     ")
        #sys.stdout.flush()
        print(compass)
        time.sleep(0.5)

