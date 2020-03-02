from qcodes.utils.validators import Numbers
import socket

from qcodes.instrument import ip

class RCDAT800090(ip.IPInstrument):
    def __init__(self, name, address):
        super().__init__(name, address, port=23, terminator="\r\n")

        self._buffer_size = 512

        self.add_parameter(name="attenuation",
                           label="Att",
                           unit="dB",
                           get_cmd=":ATT?",
                           set_cmd=":SETATT={}",
                           vals=Numbers(min_value=0, max_value=90))
        self._recv()


def main():

    va = VariableAttenuator("VA", "10.21.42.143")
    print("PRE", va.get("attenuation"))
    va.set("attenuation", 70)
    print("POST", va.get("attenuation"))

    va.close()


if __name__ == "__main__":
    main()