import React from 'react';
import ChatBot from './ChatBot';
import './ChatBot.css';

const ChatInterface = ({ position = 'bottom-right' }) => {
  const [isOpen, setIsOpen] = React.useState(false);

  if (!isOpen) {
    return (
      <button
        className="chat-toggle-button"
        onClick={() => setIsOpen(true)}
      >
        💬 Open Chat
      </button>
    );
  }

  return (
    <div className={`chat-interface chat-${position}`}>
      <div className="chat-header-controls">
        <h3>Robotics Book Assistant</h3>
        <button
          className="close-button"
          onClick={() => setIsOpen(false)}
        >
          ×
        </button>
      </div>
      <ChatBot />
    </div>
  );
};

export default ChatInterface;