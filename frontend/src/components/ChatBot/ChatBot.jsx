import React, { useState, useRef, useEffect } from 'react';
import { useColorMode } from '@docusaurus/theme-common';
import { API_ENDPOINTS, DEMO_MODE, safeFetch } from '../../config/api';
import styles from './ChatBot.module.css';

// Demo responses for when backend is unavailable
const DEMO_RESPONSES = {
  ros2: "ROS 2 (Robot Operating System 2) is the middleware framework for robotics. It provides communication infrastructure using DDS (Data Distribution Service), allowing nodes to communicate via topics, services, and actions. Key concepts include nodes, publishers, subscribers, and the rclpy Python library for creating ROS 2 applications.",
  simulation: "Simulation environments like Gazebo and Unity allow you to test robot behaviors virtually before deploying to real hardware. Gazebo provides physics simulation with ODE/Bullet engines, while Unity offers high-fidelity rendering for HRI (Human-Robot Interaction) scenarios.",
  isaac: "NVIDIA Isaac is a platform for accelerated robotics development. It includes Isaac Sim for photorealistic simulation, Isaac ROS for hardware-accelerated perception, and integrates with Nav2 for autonomous navigation. Isaac enables synthetic data generation for training AI models.",
  vla: "VLA (Vision-Language-Action) models bridge natural language understanding with robotic actions. They use models like OpenAI Whisper for speech recognition and LLMs for cognitive planning, translating voice commands into ROS 2 action sequences.",
  default: "I'm the Physical AI & Robotics Book Assistant! This textbook covers ROS 2, simulation (Gazebo/Unity), NVIDIA Isaac, VLA systems, and more. The backend is currently offline, but I can provide basic information from the book content. Try asking about ROS 2, simulation, Isaac, or VLA!"
};

function getDemoResponse(query) {
  const q = query.toLowerCase();
  if (q.includes('ros') || q.includes('node') || q.includes('topic') || q.includes('rclpy')) {
    return DEMO_RESPONSES.ros2;
  }
  if (q.includes('simulation') || q.includes('gazebo') || q.includes('unity') || q.includes('physics')) {
    return DEMO_RESPONSES.simulation;
  }
  if (q.includes('isaac') || q.includes('nvidia') || q.includes('nav2') || q.includes('vslam')) {
    return DEMO_RESPONSES.isaac;
  }
  if (q.includes('vla') || q.includes('voice') || q.includes('whisper') || q.includes('language')) {
    return DEMO_RESPONSES.vla;
  }
  return DEMO_RESPONSES.default;
}

const ChatBot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);
  const { colorMode } = useColorMode();

  // Function to get currently selected text
  useEffect(() => {
    const handleSelection = () => {
      const selectedText = window.getSelection().toString().trim();
      if (selectedText) {
        setSelectedText(selectedText);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, []);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Add user message
    const userMessage = { id: Date.now(), text: inputValue, sender: 'user' };
    setMessages(prev => [...prev, userMessage]);
    const currentQuery = inputValue;
    setInputValue('');
    setIsLoading(true);

    try {
      let botText = '';
      let sources = [];
      let confidence = 0;

      if (DEMO_MODE) {
        // Simulate API delay for demo mode
        await new Promise(resolve => setTimeout(resolve, 800));
        botText = getDemoResponse(currentQuery);
        confidence = 0.85;
      } else {
        // Real API call to backend /api/chat
        const response = await safeFetch(API_ENDPOINTS.chat, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            query: currentQuery,
            selection_context: selectedText || null,
            session_id: 'docusaurus-user-session'
          })
        });

        if (response.ok) {
          const data = await response.json();
          botText = data.response;
          sources = data.sources || [];
          confidence = data.confidence || 0;
        } else {
          // Fallback to demo if API fails
          botText = getDemoResponse(currentQuery);
          confidence = 0.7;
        }
      }

      const botMessage = {
        id: Date.now() + 1,
        text: botText,
        sender: 'bot',
        sources: sources,
        confidence: confidence
      };
      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      const errorMessage = {
        id: Date.now() + 1,
        text: getDemoResponse(currentQuery),
        sender: 'bot',
        error: false
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
      setSelectedText(''); // Clear selected text after use
    }
  };

  return (
    <>
      {/* Floating Action Button */}
      <button
        className={styles.fab}
        onClick={toggleChat}
        aria-label={isOpen ? 'Close chat' : 'Open chat'}
      >
        {isOpen ? '✕' : '💬'}
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.window}>
          <div className={styles.header}>
            <div className={styles.headerInfo}>
              <span className={styles.icon}>🤖</span>
              <h3 className={styles.title}>Book Bot</h3>
            </div>
            <button
              type="button"
              className={styles.closeButton}
              onClick={(e) => {
                e.stopPropagation();
                e.preventDefault();
                setIsOpen(false);
              }}
              aria-label="Close chat"
            >
              ✕
            </button>
          </div>

          <div className={styles.messages}>
            {messages.length === 0 ? (
              <div className={styles.welcome}>
                <p><strong>Hello! I'm your Robotics Book Assistant.</strong></p>
                <p>Ask me questions about the content, or select text on the page and ask about it specifically!</p>
              </div>
            ) : (
              messages.map((message) => (
                <div
                  key={message.id}
                  className={`${styles.message} ${message.sender === 'user' ? styles.userMessage : styles.botMessage}`}
                >
                  <p>{message.text}</p>
                  {message.sources && message.sources.length > 0 && (
                    <div className={styles.sources}>
                      <small>Sources: {message.sources.map(s => `Ch. ${s.chapter}`).join(', ')}</small>
                    </div>
                  )}
                </div>
              ))
            )}
            {isLoading && (
              <div className={`${styles.message} ${styles.botMessage}`}>
                <p className={styles.typing}>Thinking...</p>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <form className={styles.form} onSubmit={handleSubmit}>
            <input
              className={styles.input}
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Ask a question about robotics..."
              disabled={isLoading}
            />
            <button className={styles.sendButton} type="submit" disabled={isLoading || !inputValue.trim()}>
              {isLoading ? 'Sending...' : 'Send'}
            </button>
          </form>
        </div>
      )}
    </>
  );
};

export default ChatBot;