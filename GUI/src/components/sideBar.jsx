import { GiGamepad } from "react-icons/gi";
import { FaList } from "react-icons/fa";
import BarElement from "./BarElement";
import { useState } from "react";
import { core } from "../store/core";

const SideBar = () => {
    const [menu, setMenu] = useState("Remote");
    const { setmenu, menuOpened, setmenuOpened } = core();
  
    const cn = menuOpened
      ? "w-64 h-full bg-slate-900 animate-[sslidein_1s_forwards] overflow-hidden"
      : "w-0 h-full bg-slate-900 animate-[sslideout_1s_forwards] overflow-hidden";
  
    const handleChangeMenu = (menu) => {
      setmenu(menu);
      setMenu(menu);
      setmenuOpened(false); // Close sidebar
    };
  
    return (
      <div className={cn}>
        <div className="w-full flex items-center justify-center h-20">
          <h1 className="text-4xl text-white">Menu</h1>
        </div>
        <div className="mt-10 text-white">
          <BarElement
            icon={<GiGamepad />}
            text={"Remote Control"}
            name={"Remote"}
            handle={handleChangeMenu}
            current={menu}
          />
          <BarElement
            icon={<FaList />}
            text={"Task"}
            name={"Task"}
            handle={handleChangeMenu}
            current={menu}
          />
        </div>
      </div>
    );
};

export default SideBar;
  