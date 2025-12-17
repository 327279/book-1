import React from 'react';
import './ChatBot.css';

class ChatInterface extends React.Component {
  constructor(props) {
    super(props);
    this.state = {
      isOpen: false,
      messages: [],
      inputValue: '',
      isLoading: false
    };
  }

  toggleChat = () => {
    this.setState(prevState => ({ isOpen: !prevState.isOpen }));
  }

  handleInputChange = (e) => {
    this.setState({ inputValue: e.target.value });
  }

  getDemoResponse = (query) => {
    const q = query.toLowerCase();
    if (q.includes('ros')) return "ROS 2 is the robotics middleware framework. It provides communication infrastructure for building robot applications.";
    if (q.includes('simulation') || q.includes('gazebo')) return "Simulation allows testing robots virtually before deployment. Gazebo is a popular physics-based simulator.";
    if (q.includes('isaac') || q.includes('nvidia')) return "NVIDIA Isaac is a platform for accelerated robotics development with photorealistic simulation.";
    if (q.includes('vla')) return "VLA (Vision-Language-Action) models bridge natural language understanding with robotic actions.";
    return "I'm the Physical AI & Robotics Assistant! Ask me about ROS 2, simulation, NVIDIA Isaac, VLA systems, or any robotics topic from the book.";
  }

  handleSubmit = (e) => {
    e.preventDefault();
    const { inputValue, isLoading, messages } = this.state;

    if (!inputValue.trim() || isLoading) return;

    const userMessage = { id: Date.now(), text: inputValue, sender: 'user' };
    this.setState({
      messages: [...messages, userMessage],
      inputValue: '',
      isLoading: true
    });

    setTimeout(() => {
      const botMessage = {
        id: Date.now() + 1,
        text: this.getDemoResponse(inputValue),
        sender: 'bot'
      };
      this.setState(prevState => ({
        messages: [...prevState.messages, botMessage],
        isLoading: false
      }));
    }, 800);
  }

  render() {
    const { isOpen, messages, inputValue, isLoading } = this.state;

    return (
      <div>
        <button
          className="chat-fab"
          onClick={this.toggleChat}
          aria-label={isOpen ? "Close chat" : "Open chat"}
        >
          {isOpen ? '✕' : '💬'}
        </button>

        {isOpen && (
          <div className="chatbot-window">
            <div className="chatbot-header">
              <div className="chatbot-header-info">
                <span className="chatbot-icon">🤖</span>
                <span className="chatbot-title">AI Assistant</span>
              </div>
              <button className="chatbot-close" onClick={this.toggleChat}>
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
            </div>

            <form className="chatbot-form" onSubmit={this.handleSubmit}>
              <input
                type="text"
                value={inputValue}
                onChange={this.handleInputChange}
                placeholder="Ask a question..."
                disabled={isLoading}
              />
              <button type="submit" disabled={isLoading || !inputValue.trim()}>
                Send
              </button>
            </form>
          </div>
        )}
      </div>
    );
  }
}

export default ChatInterface;