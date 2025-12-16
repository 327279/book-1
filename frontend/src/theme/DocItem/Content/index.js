import React, { useState } from 'react';
import Content from '@theme-original/DocItem/Content';
import { useAuth } from '../../../components/Auth/AuthContext';
import styles from './styles.module.css';

// Mock API call to backend agent
const callAgentSkill = async (skill, text, context = {}) => {
    // In a real app, this would POST to http://localhost:8000/api/v1/agent/skill
    console.log(`Calling skill: ${skill}`, context);

    if (skill === 'translate_urdu') {
        return '🤖 [AI Translation]: ' + text.substring(0, 100) + '... (Urdu Translation Mock)';
    }
    if (skill === 'personalize') {
        return '🤖 [AI Personalization]: Based on your background in ' + context.software_bg + ', think of this concept like...';
    }
    return text;
};

export default function DocItemContentWrapper(props) {
    const { user } = useAuth();
    const [personalizedContent, setPersonalizedContent] = useState(null);
    const [translatedContent, setTranslatedContent] = useState(null);
    const [loading, setLoading] = useState(false);

    const handlePersonalize = async () => {
        if (!user) {
            alert('Please sign in to use Personalization.');
            return;
        }
        setLoading(true);
        // Simulate API delay
        setTimeout(async () => {
            const result = await callAgentSkill('personalize', 'Current Chapter Content', user.profile);
            setPersonalizedContent(result);
            setLoading(false);
        }, 1000);
    };

    const handleTranslate = async () => {
        setLoading(true);
        setTimeout(async () => {
            const result = await callAgentSkill('translate_urdu', 'Current Chapter Content');
            setTranslatedContent(result);
            setLoading(false);
        }, 1000);
    };

    return (
        <>
            <div className="row margin-bottom--md">
                <div className="col">
                    {user ? (
                        <button
                            onClick={handlePersonalize}
                            disabled={loading}
                            className="button button--secondary button--sm margin-right--sm">
                            ✨ Personalize for {user.profile.software_bg}
                        </button>
                    ) : (
                        <span className="badge badge--warning margin-right--sm">Sign in to Personalize</span>
                    )}

                    <button
                        onClick={handleTranslate}
                        disabled={loading}
                        className="button button--outline button--primary button--sm">
                        🇵🇰 Translate to Urdu
                    </button>
                </div>
            </div>

            {loading && <div className="alert alert--info">🤖 AI Agent is working on your request...</div>}

            {personalizedContent && (
                <div className="alert alert--success margin-bottom--md">
                    <strong>Personalized Explanation:</strong> {personalizedContent}
                    <button
                        className="close"
                        onClick={() => setPersonalizedContent(null)}
                        style={{ float: 'right', background: 'none', border: 'none', cursor: 'pointer' }}>
                        ×
                    </button>
                </div>
            )}

            {translatedContent && (
                <div className="alert alert--info margin-bottom--md" dir="rtl" style={{ fontFamily: 'Noto Nastaliq Urdu, serif' }}>
                    <strong>اردو ترجمہ:</strong><br />
                    {translatedContent}
                    <button
                        className="close"
                        onClick={() => setTranslatedContent(null)}
                        style={{ float: 'left', background: 'none', border: 'none', cursor: 'pointer' }}>
                        ×
                    </button>
                </div>
            )}

            <Content {...props} />
        </>
    );
}
