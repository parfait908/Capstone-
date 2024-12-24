import { TbBaselineDensitySmall } from "react-icons/tb";
import { clientStore } from "../store/clientStore";


const NavBar = (props) => {
    const {menu, menuOpened, setmenuOpened} =clientStore();
    const handleMenu = () => {
        setmenuOpened(!menuOpened);
    }
    return (
        <div className="w-full h-16 bg-slate-900 flex flex-row items-center">
            <button onClick={handleMenu}>
                <TbBaselineDensitySmall className="size-8 ml-2"/>
            </button>
            <h1 className="ml-2">{menu}</h1>
        </div>
    );
}

export default NavBar;