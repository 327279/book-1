import React from 'react';
import clsx from 'clsx';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useBaseUrlUtils } from '@docusaurus/useBaseUrl';
import Navbar from '@theme/Navbar';
import Footer from '@theme/Footer';
import LayoutProvider from '@theme/Layout/Provider';
import ErrorBoundary from '@docusaurus/ErrorBoundary';
import { PageMetadata, ThemeClassNames } from '@docusaurus/theme-common';
import { translate } from '@docusaurus/Translate';
import ChatInterface from '@site/src/components/ChatBot/ChatInterface';

import styles from './styles.module.css';

function Layout(props) {
  const {
    children,
    noFooter,
    wrapperClassName,
    pageClassName,
  } = props;

  const location = useLocation();
  const { siteConfig } = useDocusaurusContext();
  const { withBaseUrl } = useBaseUrlUtils();

  return (
    <LayoutProvider>
      <PageMetadata>
      </PageMetadata>
      <div className={clsx(ThemeClassNames.wrapper.main, wrapperClassName)}>
        <Navbar />
        <main className={clsx(pageClassName)} itemProp="mainContentOfPage">
          {children}
        </main>
        <ChatInterface />
      </div>
      {!noFooter && <Footer />}
    </LayoutProvider>
  );
}

export default function(props) {
  return (
    <ErrorBoundary
      fallback={(params) => (
        <Layout {...props}>
          <div>
            <h1>{translate({ message: 'This page has crashed.' })}</h1>
            <p>{params.error.message}</p>
          </div>
        </Layout>
      )}
    >
      <Layout {...props} />
    </ErrorBoundary>
  );
}