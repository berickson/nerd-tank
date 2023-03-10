var mqtt    = require('mqtt');

const fs = require('fs');

secrets = require("./secrets.js");



console.log("connecting client to " + secrets.mqtt_server_address);
options = secrets.mqtt_server_options;
options.clientId = "json-client";
let client  = mqtt.connect(secrets.mqtt_server_address, secrets.mqtt_server_options);
console.log("past connecting client");


client.subscribe("#", opts={qos:2})
client.on('connect', () => console.log("connected"));
client.on("message", function(topic,payload) {
    filename = topic.replace("/","-") + ".csv";
    filepath = "data/"+filename;
    let d = new Date();
    let log_string = [topic, d.toISOString(), payload.toString()].join(",")
    console.log(log_string);
    fs.appendFile(filepath,log_string+"\n", function (err) {
      if (err) throw err;
    });
      
})