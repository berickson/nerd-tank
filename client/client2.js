var mqtt    = require('mqtt');
const fs = require('fs');
secrets = require("./secrets.js");



console.log("connecting client to " + secrets.mqtt_server_address);
let client  = mqtt.connect(secrets.mqtt_server_address, secrets.mqtt_server_options);
console.log("past connecting client");


// client.subscribe("#")
client.subscribe("d07d4ffa12f4/reading", {qos: 0})

client.on('connect', () => console.log("connected"));
client.on("message", function(topic,payload) {
    message = {}

    filename = topic.replace("/","-") + ".json_lines";
    filepath = "data/"+filename;
    
    let d = new Date();
    message["topic"]=topic
    message["received"] = d.toISOString()
    try {
        message["payload"]=JSON.parse(payload);
    }
    catch {
        message["payload"]=payload;
    }
    
    log_string = JSON.stringify(message);
    console.log(log_string);

    fs.appendFile(filepath,log_string+"\n", function (err) {
     if(err) {
            console.log("Error writing to "+filepath);
        }
     });
      
})