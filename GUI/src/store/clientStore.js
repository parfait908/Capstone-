import {create} from 'zustand';
import io from 'socket.io-client';
import ROSLIB from 'roslib';
import toast from 'react-hot-toast';

export const clientStore = create((set,get) => ({
    connectedSocket : false,
    connectedRos : false,
    menu : "Dashboard",
    menuOpened : false,
    socket : null,
    ros : null,
    cameraData : null,
    camera_qr_data : null,
    speed : 0,
    mapData : null,
    gear : 0,
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

          socket.on('gear', (data) => {
            set({gear : data.gear})
          });
      }
    },
    sendDirection: async function (direction) {
      const {socket } = get()
      socket.emit("moveDirection", { direction: direction });
    },
    initializeRos : function(){

      const rs = new ROSLIB.Ros({
        url : 'ws://localhost:9090'
      })

      rs.on('connection', () => {
        set({connectedRos : true})
      });

      rs.on('error', (error) => {
          set({connectedRos : false})
      });

      rs.on('close', () => {
          set({connectedRos : false})
      });
      set({ros : rs})
    },

    camera_feed : async function(data){

        set({cameraData : data.image})
    },
    camera_qr_feed : async function(data){

        set({camera_qr_data : data.image})
    },

    map_feed : async function(data){
        set({mapData : data.image})
    },

    sendTask : async function(task){
      const {socket} = get()
      socket.emit('task', {task : task})
    },

    mapping :async function(){
      toast('Mapping started', {
        icon: '⚙️',
      });
      const {socket} = get()
      const task = {
        task_name : 'mapping',
        params : {}
      }
      socket.emit("robot_task", {task : task})
    }

}));

