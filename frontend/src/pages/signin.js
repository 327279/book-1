import React from 'react';
import Layout from '@theme/Layout';
import SignInForm from '../components/Auth/SignInForm';

export default function SignInPage() {
    return (
        <Layout
            title="Sign In"
            description="Sign in to your Physical AI learning account">
            <main className="container margin-vert--xl">
                <div className="row">
                    <div className="col col--6 col--offset-3">
                        <SignInForm />
                    </div>
                </div>
            </main>
        </Layout>
    );
}
