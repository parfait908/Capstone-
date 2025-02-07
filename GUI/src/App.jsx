import react, { useEffect } from "react";
import SideBar from "./components/sideBar";
import "./css/main.css";
import { BrowserRouter } from 'react-router-dom';
import MainLayout from "./components/MainLayout";
import { core } from "./store/core";
import Loarder from "./components/Loader";
import {Toaster} from "react-hot-toast"

function App() {
  const {initializeSocket, connectedSocket, connectedRos}= core()
  useEffect(function(){
    initializeSocket()
  },[initializeSocket])

  if(!(connectedSocket  )){
    return <Loarder/>
  }

  return (
    <BrowserRouter>
    <div className="flex flex-row w-full h-full">
        <SideBar />
        <MainLayout />
    </div>
    <Toaster></Toaster>
    </BrowserRouter>
  );
}

export default App;
