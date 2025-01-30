import logging
import threading
# # the import way of manual compilation
# from zmotion_py import (
# # the import way of runtime compilation. change to your own path.
from runtime_compilation.zmotion_py import (
    Handle_Wrapper,
    zmotion_close,
    zmotion_direct,
    zmotion_execute,
    zmotion_init,
    zmotion_open_ethernet,
)


class MotionControlCard:

    def __init__(self, ip_address: str):
        self._ip_address = "0.0.0.0"
        zmotion_init()
        self._controller_handle = Handle_Wrapper()
        connect_success = zmotion_open_ethernet(ip_address, self._controller_handle)
        if not connect_success == 0:
            raise ConnectionError(f"Failed to connect to motion controller at ip: {ip_address}.")
        logging.info(f"Connected to motion control card {ip_address}")
        self._cmd_lock = threading.Lock()
        self._closed = False

    def __del__(self):
        self.close()

    def close(self):
        if not self._closed:
            self._closed = True
            zmotion_close(self._controller_handle)

    def send_cmd(
        self,
        cmd_lock: threading.Lock,
        cmd: str,
        direct_or_execute: str = "direct"
    ) -> str:
        """Send a command to the motion command card practicality.

        This is a core function.
        Now, the duration of sending one command to the motion control card is about 0.15ms - 0.35ms.

        Parameters
        ----------
        cmd: str
            The sent command.
        direct_or_execute: str
            "direct": direct command interface, used for motion functions or some io operation.
            "execute": universal command execution interface.

        Returns
        -------
        response : str
            The response detail.
        """
        with cmd_lock:
            if direct_or_execute == "direct":
                # direct command interface, only supports motion functions, parameters and array variables configuration
                success, response = zmotion_direct(controller_handle, cmd)
            else:
                # Universal command execution interface, getting block when the controller is not buffered
                success, response = zmotion_execute(controller_handle, cmd)

        if not success == 0:
            raise ConnectionError(
                f"{'ZMC_DirectCommand' if direct_or_execute == 'direct' else 'ZMC_Execute'} failed to send {cmd} "
                f"with {success} status. Its response is {response}."
            )
        return response

    def get_in(self, in_num: int) -> int:
        """Get the value of the specified input."""
        return int(self.send_cmd(f"?IN({in_num})"))


if __name__ == "__main__":
    card = MotionControlCard(ip_address="192.168.0.11")
    logging.info(f"IN1 value: {card.get_in(1)}")
