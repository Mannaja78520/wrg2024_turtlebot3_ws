import React, { useEffect, useRef, useState } from 'react';
import { Terminal } from 'xterm';
import 'xterm/css/xterm.css';
import ROSLIB from 'roslib';
import './App.css';

const App = () => {
  const terminalRef = useRef(null);
  const xtermRef = useRef(null);
  const [ros, setRos] = useState(null);
  const [connected, setConnected] = useState(false);
  const [message, setMessage] = useState('');
  const [buttonState, setButtonState] = useState('Init'); // Init, Retry, Start

  useEffect(() => {
    const terminalInstance = new Terminal({
      theme: {
        background: '#1e1e1e',
        foreground: '#d4d4d4',
        cursor: '#ffffff',
      },
      fontFamily: 'monospace',
      fontSize: 16,
    });
    terminalInstance.open(terminalRef.current);
    terminalInstance.writeln('Terminal initialized.');
    xtermRef.current = terminalInstance;

    const rosbridgeUrl = `ws://${window.location.hostname}:9090`;
    const rosInstance = new ROSLIB.Ros({ url: rosbridgeUrl });

    rosInstance.on('connection', () => {
      console.log('Connected to ROS bridge');
      setConnected(true);
      xtermRef.current.writeln('Connected to ROS bridge');
    });

    rosInstance.on('error', (error) => {
      console.error('Error connecting to ROS bridge:', error);
      setConnected(false);
      xtermRef.current.writeln('Error connecting to ROS bridge');
    });

    rosInstance.on('close', () => {
      console.log('Connection closed');
      setConnected(false);
      xtermRef.current.writeln('Connection closed');
    });

    setRos(rosInstance);

    return () => {
      rosInstance.close();
    };
  }, []);

  useEffect(() => {
    if (ros) {
      const chatterTopic = new ROSLIB.Topic({
        ros,
        name: '/chatter',
        messageType: 'std_msgs/String'
      });

      chatterTopic.subscribe((message) => {
        const logMessage = `Received message from /chatter: ${message.data}`;
        console.log(logMessage);
        xtermRef.current.writeln(logMessage);
        terminalRef.current.scrollTop = terminalRef.current.scrollHeight; // Auto-scroll to bottom
      });

      const locationTopic = new ROSLIB.Topic({
        ros,
        name: '/challenge_location',
        messageType: 'std_msgs/String'
      });

      locationTopic.subscribe((message) => {
        const logMessage = `Received message from /challenge_location: ${message.data}`;
        console.log(logMessage);
        xtermRef.current.writeln(logMessage);
        terminalRef.current.scrollTop = terminalRef.current.scrollHeight; // Auto-scroll to bottom
      });

      return () => {
        chatterTopic.unsubscribe();
        locationTopic.unsubscribe();
      };
    }
  }, [ros]);

  const handleButtonClick = (state) => {
    if (ros) {
      const robotStateTopic = new ROSLIB.Topic({
        ros,
        name: '/robot_state',
        messageType: 'std_msgs/String'
      });

      const messageToSend = state;
      robotStateTopic.publish(new ROSLIB.Message({ data: messageToSend }));
      const logMessage = `Sent "${messageToSend}" to /robot_state`;
      console.log(logMessage);
      xtermRef.current.writeln(logMessage);
      terminalRef.current.scrollTop = terminalRef.current.scrollHeight; // Auto-scroll to bottom

      if (state === 'Init') {
        setButtonState('Start');
      } else if (state === 'Start') {
        setButtonState('Retry');
      } else if (state === 'Retry') {
        setButtonState('Init');
      }
    }
  };

  const handleInputChange = (event) => {
    setMessage(event.target.value);
  };

  const handlePublish = () => {
    if (ros && message) {
      const challengeLocationTopic = new ROSLIB.Topic({
        ros,
        name: '/challenge_location',
        messageType: 'std_msgs/String'
      });

      challengeLocationTopic.publish(new ROSLIB.Message({ data: message }));
      const logMessage = `Published "${message}" to /challenge_location`;
      console.log(logMessage);
      xtermRef.current.writeln(logMessage);
      setMessage('');
      terminalRef.current.scrollTop = terminalRef.current.scrollHeight; // Auto-scroll to bottom
    }
  };

  return (
    <div className="App">
      <div className="terminal-container">
        <div className="terminal" ref={terminalRef}></div>
      </div>
      <div className="controls">
        <button
          onClick={() => handleButtonClick('Init')}
          disabled={buttonState !== 'Init'}
        >
          Init
        </button>
        <button
          onClick={() => handleButtonClick('Retry')}
          disabled={buttonState !== 'Start'}
        >
          Retry
        </button>
        <button
          onClick={() => handleButtonClick('Start')}
          disabled={buttonState !== 'Retry'}
        >
          Start
        </button>
      </div>
      <div className="publish-container">
        <input
          type="text"
          value={message}
          onChange={handleInputChange}
          placeholder="Message to publish"
        />
        <button onClick={handlePublish}>Publish</button>
      </div>
    </div>
  );
};

export default App;
