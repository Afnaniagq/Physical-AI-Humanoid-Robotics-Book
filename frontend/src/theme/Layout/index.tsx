import React from 'react';
import Layout from '@theme-original/Layout';
import Chatbot from '@site/src/components/Chatbot';
import type {Props} from '@theme/Layout';

export default function LayoutWrapper(props: Props): JSX.Element {
  return (
    <Layout {...props}>
      {props.children}
      <div style={{
        position: 'fixed',
        bottom: '20px',
        right: '20px',
        zIndex: 1000,
        width: '400px',
        height: '500px'
      }}>
        <Chatbot />
      </div>
    </Layout>
  );
}