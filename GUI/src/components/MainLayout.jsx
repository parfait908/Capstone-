
import {BrowserRouter, Routes, Route, Navigate } from 'react-router-dom';
import Remote from './Remote';
import Task from './Task';
import Dashboard from './DashBoard';
import { clientStore } from '../store/clientStore';
import Loarder from './Loader';
import NavBar from './NavBar';
const MainLayout = function(props) {   

    const {connectedRos, connectedSocket, menuOpened} = clientStore();
    const cn = !menuOpened ? "w-full h-full    overflow-hidden" : " h-full   animate-[pslidein_1s_forwards]  overflow-hidden"
    
    return(
        
        <div className={cn}>
        <NavBar/>
        <div className='w-full h-[calc(100%-64px)]'>
            <Routes>
                <Route path="/" element={connectedSocket  ? <Dashboard /> : <Loarder/>} />
                <Route path="/remote" element={connectedSocket  ? <Remote /> : <Loarder/>} />
                <Route path="/task" element={connectedSocket  ? <Task /> : <Loarder/>} />
                <Route path="/dashboard" element={connectedSocket  ? <Dashboard /> : <Loarder/>} />
            </Routes>
        </div>
        

        </div>)
}

export default MainLayout;