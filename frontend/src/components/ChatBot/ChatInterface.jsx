import React, { useState, useRef, useEffect } from 'react';
import './ChatBot.css';

// Demo responses for when backend is unavailable
const DEMO_RESPONSES = {
  ros2: "ROS 2 (Robot Operating System 2) is the middleware framework for robotics. It provides communication infrastructure using DDS (Data Distribution Service), allowing nodes to communicate via topics, services, and actions.",
  simulation: "Simulation environments like Gazebo and Unity allow you to test robot behaviors virtually before deploying to real hardware. Gazebo provides physics simulation with ODE/Bullet engines.",
  isaac: "NVIDIA Isaac is a platform for accelerated robotics development. It includes Isaac Sim for photorealistic simulation, Isaac ROS for hardware-accelerated perception.",
  vla: "VLA (Vision-Language-Action) models bridge natural language understanding with robotic actions using LLMs for cognitive planning.",
  default: "I'm the Physical AI & Robotics Book Assistant! This textbook covers ROS 2, simulation (Gazebo/Unity), NVIDIA Isaac, VLA systems, and more. Try asking about ROS 2, simulation, Isaac, or VLA!"
};

function getDemoResponse(query) {
  const q = query.toLowerCase();
  if (q.includes('ros') || q.includes('node') || q.includes('topic')) return DEMO_RESPONSES.ros2;
  if (q.includes('simulation') || q.includes('gazebo') || q.includes('unity')) return DEMO_RESPONSES.simulation;
  if (q.includes('isaac') || q.includes('nvidia') || q.includes('nav2')) return DEMO_RESPONSES.isaac;
  if (q.includes('vla') || q.includes('voice') || q.includes('language')) return DEMO_RESPONSES.vla;
  return DEMO_RESPONSES.default;
}

const ChatBot = ({ onClose }) => {
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    const userMessage = { id: Date.now(), text: inputValue, sender: 'user' };
    setMessages(prev => [...prev, userMessage]);
    const currentQuery = inputValue;
    setInputValue('');
    setIsLoading(true);

    // Simulate API response
    setTimeout(() => {
      const botMessage = {
        id: Date.now() + 1,
        text: getDemoResponse(currentQuery),
        sender: 'bot'
      };
      setMessages(prev => [...prev, botMessage]);
      setIsLoading(false);
    }, 800);
  };

  return (
    <div className="chatbot-window">
      <div className="chatbot-header">
        <div className="chatbot-header-info">
          <span className="chatbot-icon">🤖</span>
          <span className="chatbot-title">AI Assistant</span>
        </div>
        <button className="chatbot-close" onClick={onClose} aria-label="Close chat">
          ✕
        </button>
      </div>

      <div className="chatbot-messages">
        {messages.length === 0 ? (
          <div className="chatbot-welcome">
            <p><strong>Hi! I'm your Robotics Assistant.</strong></p>
            <p>Ask me about ROS 2, simulation, NVIDIA Isaac, or anything in the book!</p>
          </div>
        ) : (
          messages.map((msg) => (
            <div key={msg.id} className={`chatbot-message ${msg.sender}`}>
              {msg.text}
            </div>
          ))
        )}
        {isLoading && (
          <div className="chatbot-message bot">
            <span className="chatbot-typing">Thinking...</span>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      <form className="chatbot-form" onSubmit={handleSubmit}>
        <input
          type="text"
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          placeholder="Ask a question..."
          disabled={isLoading}
        />
        <button type="submit" disabled={isLoading || !inputValue.trim()}>
          Send
        </button>
      </form>
    </div>
  );
};

// Main floating chat component
const ChatInterface = () => {
  const [isOpen, setIsOpen] = useState(false);

  return (
    <>
      {/* Floating action button */}
      <button
        className="chat-fab"
        onClick={() => setIsOpen(!isOpen)}
        aria-label={isOpen ? "Close chat" : "Open chat"}
      >
        {isOpen ? '✕' : '💬'}
      </button>

      {/* Chat window */}
      {isOpen && (
        <ChatBot onClose={() => setIsOpen(false)} />
      )}
    </>
  );
};

export default ChatInterface;