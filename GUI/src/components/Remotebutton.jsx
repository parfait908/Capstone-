import { clientStore } from "../store/clientStore";
import { useRef } from "react";

const RemoteButton = function (props) {
  const { sendDirection } = clientStore();
  const intervalIdRef = useRef(null); // Ref for interval ID

  const startSendingDirection = (direction) => {
    sendDirection(direction); // Immediately send the first direction
    intervalIdRef.current = setInterval(() => sendDirection(direction), 700); // Repeat every 700ms
  };

  const stopSendingDirection = () => {
    if (intervalIdRef.current) {
      clearInterval(intervalIdRef.current);
      intervalIdRef.current = null;
    }
  };

  const { direction } = props;
  const directionTextMap = {
    LEFT: "←",
    RIGHT: "→",
    UP: "↑",
    DOWN: "↓",
  };
  const text = directionTextMap[direction] || "Unknown";
  const cn = `button ${direction.toLowerCase()}`;

  return (
    <button
      className={cn}
      onMouseDown={() => startSendingDirection(direction)}
      onMouseUp={stopSendingDirection}
      onMouseLeave={stopSendingDirection}
      onTouchStart={() => startSendingDirection(direction)} // Support for touch devices
      onTouchEnd={stopSendingDirection} // Stop on touch release
    >
      {text}
    </button>
  );
};

export default RemoteButton;
