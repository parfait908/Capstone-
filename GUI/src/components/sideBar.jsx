import CIcon from '@coreui/icons-react';
import * as icon from '@coreui/icons';
import { FaChartBar, FaList } from 'react-icons/fa';
import { GiGamepad } from 'react-icons/gi';
import BarElement from './BarElement';
import { useRef, useState } from 'react';
import { clientStore } from "../store/clientStore";
const SideBar = function(){

    const [menu, setMenu] = useState("Dashboard");
    let init = useRef(true)
    const {setmenu, menuOpened} = clientStore(); 
    let cn = menuOpened ? " h-full bg-slate-900  animate-[sslidein_1s_forwards]  overflow-hidden" : init.current ? "w-0 h-full bg-slate-900   overflow-hidden" : "w-0 h-full bg-slate-900  animate-[sslideout_1s_forwards] overflow-hidden" 

    

    if(init.current){
        setTimeout(() => {
            init.current = false
        }, 5000);
    }
    const handleChangeMenu = (menu) => {
        setmenu(menu);
        setMenu(menu);
    }
    return(
        <div className={cn}>
            <div className="w-full flex items-center justify-center h-20 ">
                <h1 className="text-4xl text-white">Menu</h1>
            </div>
            <div className=" mt-10 text-white">
                <BarElement icon={<FaChartBar/>} text={"Dashboard"} name={"Dashboard"} handle={handleChangeMenu} current={menu}/>
                <BarElement icon={<GiGamepad/>} text={"Remote Control"} name={"Remote"} handle={handleChangeMenu} current={menu}/>
                <BarElement icon={<FaList/>} text={"Task"} name={"Task"} handle={handleChangeMenu} current={menu}/>
            </div>

        </div>
    )

}

export default SideBar;