import React, { useEffect, useRef, useState } from 'react';
import { Terminal } from 'xterm';
import 'xterm/css/xterm.css';
import ROSLIB from 'roslib';
import createjs from 'createjs/builds/1.0.0/createjs';
import ROS2D from '../node_modules/ros2d/src/Ros2D';
import './App.css';

const App = () => {
  const terminalRef = useRef(null);
  const xtermRef = useRef(null);
  const [ros, setRos] = useState(null);
  const [connected, setConnected] = useState(false);
  const [message, setMessage] = useState('');
  const [roomValues, setRoomValues] = useState(Array(5).fill(null)); // Initial values for rooms 1-5
  const [robotState, setRobotState] = useState('None'); // Track robot state
  const [prevState, setPrevState] = useState('None'); // Track previous state to identify new state
  const [challengeLocationPublished, setChallengeLocationPublished] = useState(false); // Track if challenge location is published

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

      const robotStateTopic = new ROSLIB.Topic({
        ros,
        name: '/robot_state',
        messageType: 'std_msgs/String'
      });

      const challengeLocationTopic = new ROSLIB.Topic({
        ros,
        name: '/challenge_location',
        messageType: 'std_msgs/Int8MultiArray'
      });

      chatterTopic.subscribe((message) => {
        const logMessage = `Received message from /chatter: ${message.data}`;
        console.log(logMessage);
        xtermRef.current.writeln(logMessage);
        terminalRef.current.scrollTop = terminalRef.current.scrollHeight;
      });

      robotStateTopic.subscribe((message) => {
        const logMessage = `Received message from /robot_state: ${message.data}`;
        console.log(logMessage);
        xtermRef.current.writeln(logMessage);
        terminalRef.current.scrollTop = terminalRef.current.scrollHeight;
        setPrevState(robotState); // Save previous state before updating
        setRobotState(message.data); // Update robot state
      });

      challengeLocationTopic.subscribe((message) => {
        const logMessage = `Received message from /challenge_location: ${message.data.join(',')}`;
        console.log(logMessage);
        xtermRef.current.writeln(logMessage); // Echo to terminal
        terminalRef.current.scrollTop = terminalRef.current.scrollHeight;
      });

      return () => {
        chatterTopic.unsubscribe();
        robotStateTopic.unsubscribe();
        challengeLocationTopic.unsubscribe();
      };
    }
  }, [ros, robotState]);

  const isAllRowsSelected = () => {
    return roomValues.every(value => value !== null);
  };

  const handleRoomValueChange = (index, value) => {
    const newValues = [...roomValues];
    newValues[index] = value; // Set room value directly
    setRoomValues(newValues);
  };

  const handlePublishChallengeLocation = () => {
    if (isAllRowsSelected()) {
      const validValues = roomValues.map(value => (value === null ? 0 : value)); // Ensure values are numbers
      const uniqueValues = new Set(validValues);

      if (uniqueValues.size !== validValues.length) {
        alert('Each room must have a unique value. Please choose again.');
        return;
      }

      if (ros) {
        const challengeLocationTopic = new ROSLIB.Topic({
          ros,
          name: '/challenge_location',
          messageType: 'std_msgs/Int8MultiArray'
        });

        const messageToSend = new ROSLIB.Message({
          data: validValues
        });

        challengeLocationTopic.publish(messageToSend);
        const formattedValues = validValues.filter(value => value > 0).join(','); // Filter out zeros
        const logMessage = `Published challenge room values: ${formattedValues}`; // Adjust for 1-5
        console.log(logMessage);
        xtermRef.current.writeln(logMessage); // Echo to terminal
        terminalRef.current.scrollTop = terminalRef.current.scrollHeight;

        setChallengeLocationPublished(true); // Mark challenge location as published
      }
    } else {
      alert('All five rooms must be selected before publishing.');
    }
  };

  const handlePublishChat = () => {
    if (ros && message) {
      const chatTopic = new ROSLIB.Topic({
        ros,
        name: '/chatter',
        messageType: 'std_msgs/String'
      });

      chatTopic.publish(new ROSLIB.Message({ data: message }));
      const logMessage = `Published "${message}" to /chatter`;
      console.log(logMessage);
      xtermRef.current.writeln(logMessage);
      setMessage('');
      terminalRef.current.scrollTop = terminalRef.current.scrollHeight;
    }
  };

  const handleButtonAction = (action) => {
    if (ros) {
      const stateTopic = new ROSLIB.Topic({
        ros,
        name: '/robot_state',
        messageType: 'std_msgs/String'
      });

      stateTopic.publish(new ROSLIB.Message({ data: action }));
      const logMessage = `Published "${action}" to /robot_state`;
      console.log(logMessage);
      xtermRef.current.writeln(logMessage);
      terminalRef.current.scrollTop = terminalRef.current.scrollHeight;

      setPrevState(robotState); // Save previous state before updating
      setRobotState(action); // Update state after action
    }
  };

  const handleReset = () => {
    if (ros) {
      const stateTopic = new ROSLIB.Topic({
        ros,
        name: '/robot_state',
        messageType: 'std_msgs/String'
      });

      stateTopic.publish(new ROSLIB.Message({ data: 'None' }));
      const logMessage = 'Published "None" to /robot_state';
      console.log(logMessage);
      xtermRef.current.writeln(logMessage);
      terminalRef.current.scrollTop = terminalRef.current.scrollHeight;

      setChallengeLocationPublished(false); // Reset challenge location published status
    }
  };

  const handleResetRoomSelection = () => {
    setRoomValues(Array(5).fill(null)); // Reset room values
  };

  const getButtonClassName = (action) => {
    if (robotState === action) {
      return `button ${action.toLowerCase()} active`;
    }
    return `button ${action.toLowerCase()}`;
  };

  const getButtonState = (action) => {
    if (action === 'Init') return robotState === 'None';
    if (action === 'Start') return robotState === 'Init' && isAllRowsSelected() && challengeLocationPublished;
    if (action === 'Retry') return robotState === 'Start';
    return true;
  };

  return (
    <div className="App">
      <div className="terminal-container">
        <div className="terminal" ref={terminalRef}></div>
      </div>
      <div className="room-controls">
        {roomValues.map((value, index) => (
          <div key={index} className="room-row">
            <span>Room {index + 1}</span>
            {[...Array(5).keys()].map(i => (
              <button
                key={i}
                onClick={() => handleRoomValueChange(index, i + 1)} // Adjust for 1-5
                className={`room-button ${value === i + 1 ? 'selected' : ''}`}
              >
                {i + 1}
              </button>
            ))}
          </div>
        ))}
        <div className="publish-reset-buttons">
          <button 
            onClick={handlePublishChallengeLocation} 
            className="publish-button"
            disabled={!isAllRowsSelected()} // Disable if not all rows are selected
          >
            Publish Challenge Location
          </button>
          <button 
            onClick={handleResetRoomSelection} 
            className="reset-room-button"
          >
            Reset Room Selection
          </button>
        </div>
        <div className="button-controls">
          <button 
            onClick={() => handleButtonAction('Init')} 
            className={getButtonClassName('Init')}
            disabled={!getButtonState('Init')}
          >
            Init
          </button>
          <button 
            onClick={() => handleButtonAction('Start')} 
            className={getButtonClassName('Start')}
            disabled={!getButtonState('Start')}
          >
            Start
          </button>
          <button 
            onClick={() => handleButtonAction('Retry')} 
            className={getButtonClassName('Retry')}
            disabled={!getButtonState('Retry')}
          >
            Retry
          </button>
          <button 
            onClick={handleReset} 
            className="reset-button"
          >
            Reset
          </button>
        </div>
        <div className="state-display">
          <span className="state-label">Current robot state is:</span>
          <span className={`state-value state-${robotState.toLowerCase()}`}>{robotState}</span>
        </div>
      </div>
      <div className="message-input-container">
        <input
          type="text"
          value={message}
          onChange={(e) => setMessage(e.target.value)}
          placeholder="Type message to publish"
        />
        <button onClick={handlePublishChat} className="publish-chat-button">Publish Chat</button>
      </div>
    </div>
  );
};

export default App;