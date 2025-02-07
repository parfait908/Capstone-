import {create} from 'zustand';
import io from 'socket.io-client';
import toast from 'react-hot-toast';

export const core = create((set,get) => ({
    connectedSocket : false,
    connectedRos : false,
    menu : "Remote",
    menuOpened : false,
    socket : null,
    cameraData : null,
    camera_qr_data : null,
    speed : 0,
    mapData : null,
    gear : 0,
    zone : null,
    station : null,
    processState : "stationary",

    setmenu : (menu) => set({menu}),
    setmenuOpened : (menuOpened) => set({menuOpened}),
    initializeSocket: async function () { 
  
      if(!get().socket) {

          const socket = io('http://localhost:5000');
          set({ socket });

        
          socket.on('connect', async (data) => {
            console.log('Connected')
            set({connectedSocket : true})

          });
          
          socket.on('disconnect', async () => {
              set({connectedSocket : false})
          });

          socket.on('message', (data) => {
            console.log('Response from server:', data);
          });

          const {camera_feed} = get()
          socket.on('camera_feed',camera_feed)

          const {camera_qr_feed} = get()
          socket.on('camera_qr_feed',camera_qr_feed)

          const {map_feed} = get()
          socket.on('map_feed',map_feed)

          socket.on('speed', (data) => {
            set({speed : data.speed})
          });

          socket.on('task_response', (data) => {
            toast(data.response, {
              icon: '⚙️',
            });
          });

          socket.on('gear_state', (data) => {
            set({gear : data.gear})
          });

          socket.on('mapping_output', (data) => {
            const {serialize} = get()
            serialize(data.locations)
          });

          socket.on('processState', (data) => {
            console.log(data)
            set({processState : data.state})
          });

      }
    },
    sendDirection: async function (direction) {
      const {processState} = get()
      if (processState == "Processing"){
        return toast.error("The robot is under a process")
      }
      
      const {socket } = get()
      socket.emit("moveDirection", { direction: direction });
    },

    camera_feed : async function(data){

        set({cameraData : data.image})
    },
    camera_qr_feed : async function(data){
        console.log(data)
        set({camera_qr_data : data.image})
    },
    map_feed : async function(data){
        set({mapData : data.image})
    },
    sendTask : async function(params){
      const {socket} = get()
      const task = {
        task_name : 'carrying',
        params : Object.assign({}, ...params)
      }
      console.log(task)
      socket.emit('robot_task', {task : task})
    },
    mapping :async function(){
      
      const {socket} = get()
      const task = {
        task_name : 'mapping',
        params : {}
      }
      socket.emit("robot_task", {task : task})
    },
    serialize : async function(locations){
      console.log(locations)
      let lcs = []
      let st = []
      locations.forEach(location => {
        let nm = location.location.split("_")
        if(nm[0] == "intersection"){
          if (nm[2] == "x" || nm[2] == "y"){
              
          }else{
            let found = false
            for(let i = 0; i < lcs.length; i++){
              if (lcs[i].location == nm[1] + " " + nm[2]){
                found = true
              }
            }
            if (!found){
              lcs.push({
                location : nm[1] + " " + nm[2],
                x : location.x,
                y : location.y,
              })
            }
          }
        }else if (nm[0] == "station"){
          let found = false
            for(let i = 0; i < st.length; i++){
              if (st[i].location == nm[0] + " " + nm[1]){
                found = true
              }
            }
            if (!found){
              st.push({
              location : nm[0] + " " + nm[1],
              x : location.x,
              y : location.y,
            })
          }
        }
      });
      set({zone : lcs})
      set({station : st})
    },
    changeGear : async function(){
      const {socket, gear, processState} = get()
      if (processState == "Processing"){
        return toast.error("The robot is under a process")
      }
      console.log("changing gear")
      socket.emit("robot_gear", {gear : !gear})
    },
}));

