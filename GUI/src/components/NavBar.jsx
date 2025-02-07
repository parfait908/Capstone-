import { TbBaselineDensitySmall } from "react-icons/tb";
import { core } from "../store/core";


const NavBar = (props) => {
    const {menu, menuOpened, setmenuOpened} =core();
    const handleMenu = () => {
        setmenuOpened(!menuOpened);
    }
    return (
        <div className="w-full h-16 bg-slate-900 flex flex-row items-center">
            <button onClick={handleMenu}>
                <TbBaselineDensitySmall className="size-8 ml-2"/>
            </button>
            <h1 className="ml-2 text-white">{menu}</h1>
        </div>
    );
}

export default NavBar;